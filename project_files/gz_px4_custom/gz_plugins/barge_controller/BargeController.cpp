/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 *
 * This customized controller is designed to control a moving platform with more
 * degrees of freedom than the standard MovingPlatformController. Need a PD
 * position controller for all 6 axes. Noise is not necessary but can be added.
 * Must also be some way of publishing a given trajectory to the platform through
 * ROS2.
 *
 *****************************************************************************/

#include "BargeController.hpp"

 using namespace custom;

 // Register the plugin
 GZ_ADD_PLUGIN(
    BargeController,
    gz::sim::System,
    gz::sim::ISystemPreUpdate,
    gz::sim::ISystemConfigure
 )

 // Configure system - runs on startup
 void BargeController::Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &eventMgr)
{

    _entity = entity;
    _model = gz::sim::Model(entity);

    const std::string link_name = sdf->Get<std::string>("link_name");
    _link_entity = _model.LinkByName(ecm, link_name);

    if (!_link_entity) {
		throw std::runtime_error("BargeController::Configure: Link \"" + link_name + "\" was not found. "
					 "Please ensure that your model contains the corresponding link.");
	}

    _link = gz::sim::Link(_link_entity);

    // Need to report linear & angular velocity
    _link.EnableVelocityChecks(ecm, true);

    _world_entity = gz::sim::worldEntity(ecm);
    _world = gz::sim::World(_world_entity);

    // getVehicleModelName();
    _startup_timer = gz::common::Timer();
    _startup_timer.Start();

    // Initialize node + get setpoints from function
    {
		std::string topic = "barge/cmd";
		// Subscribe to node and register a callback
		if (!_node.Subscribe(topic, &BargeController::bargeCallback, this))
		{
			gzerr << "Error subscribing to topic [" << topic << "]" << std::endl;
		}
	}

    // Get gravity, model mass, platform height.
    {
        const auto gravity = _world.Gravity(ecm);

        if (gravity.has_value()) {
            _gravity = gravity.value().Z();
        } else {
            gzwarn << "Unable to get gazebo world gravity. Keeping default of " << _gravity << std::endl;
        }

        auto inertial_component = ecm.Component<gz::sim::components::Inertial>(_link_entity);

		if (inertial_component) {
			_platform_mass = inertial_component->Data().MassMatrix().Mass();
			_platform_diag_moments = inertial_component->Data().MassMatrix().DiagonalMoments();

		} else {
			gzwarn << "Unable to get inertial component for link " << link_name << ". Keeping default mass of " << _platform_mass <<
			       std::endl;
		}
    }
}

void BargeController::PreUpdate(const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &ecm)
{
	getPlatformState(ecm);

	// Gets change in dt in last simulation step (in nanoseconds?)
	// const double dt_sec = std::chrono::duration<double>(_info.dt).count();

	// Wait for vehicle to spawn
	const bool vehicle_has_spawned = 0 != _world.ModelByName(ecm, _vehicle_model_name);
	const bool keep_stationary = _wait_for_vehicle_spawned && !vehicle_has_spawned;

	// Update wrench command, send wrench command
	updateWrenchCommand(_position_sp, _orientation_sp, keep_stationary);
	sendWrenchCommand(ecm);
}

void BargeController::updateWrenchCommand(const gz::math::Vector3d &velocity_setpoint,
					const gz::math::Quaterniond &orientation_setpoint,
					const bool keep_stationary)
{

	// Account for gravity on platform
	const gz::math::Vector3d normal_force(0., 0., -_gravity * _platform_mass);
	_force = normal_force;


	// Feedback terms for platform position
	{
		const gz::math::Vector3d pos_gains = 98600 * 3 * gz::math::Vector3d(1., 1., 1.);
		const gz::math::Vector3d vel_gains = 44000 * 2 * gz::math::Vector3d(1., 1., 1.);

		const gz::math::Vector3d platform_position_setpoint = _position_sp;
		const gz::math::Vector3d platform_velocity_setpoint = gz::math::Vector3d::Zero;
		// Can add feedforward later - lol

		const gz::math::Vector3d platform_pos_error = (_platform_position - platform_position_setpoint);
		const gz::math::Vector3d platform_vel_error = (_platform_velocity - platform_velocity_setpoint);

		// Note * is elementwise multiplication
		const gz::math::Vector3d feedback_force = - pos_gains * platform_pos_error - vel_gains * platform_vel_error;
		const float max_accel = 7.0;

		const gz::math::Vector2d _force_xy = gz::math::Vector2d(feedback_force.X(), feedback_force.Y());
		const float accel_xy = _force_xy.Length() / _platform_mass;

		if (accel_xy > max_accel) {
			const float scaling = max_accel / accel_xy;
			_force += feedback_force * gz::math::Vector3d(scaling, scaling, 1.);

		} else {
			_force += feedback_force;
		}
	}

	// Feedback terms for platform orientation - adapted from moving platform controller (quaternion weirdness)
	// Combining ideas from:
	//  - Eq. 23 in Nonlinear Quadrocopter Attitude Control (Brescianini, Hehn, D'Andrea)
	//    https://www.research-collection.ethz.ch/handle/20.500.11850/154099
	//  - Eq. 20 in Full Quaternion Based Attitude Control for a Quadrotor (Fresk, Nikolakopoulos)
	//    https://www.diva-portal.org/smash/get/diva2:1010947/FULLTEXT01.pdf

	{
		const gz::math::Quaterniond attitude_err = _platform_orientation * orientation_setpoint.Inverse();

		// With the factors of 1. having units of 1 / (m rad) and s / (m rad), respectively
		const gz::math::Vector3d attitude_p_gain = 10 * _platform_diag_moments; // [N m / rad]
		const gz::math::Vector3d attitude_d_gain = 5 * _platform_diag_moments; // [N m / (rad/s)]

		const double sgn = attitude_err.W() > 0. ? 1. : -1.;
		gz::math::Vector3d attitude_err_imag = sgn * gz::math::Vector3d(attitude_err.X(), attitude_err.Y(), attitude_err.Z());

		// Factor of 2 to convert quaternion error to rad
		_torque = -(2. * attitude_p_gain * attitude_err_imag + attitude_d_gain * _platform_angular_velocity);
	}
}

void BargeController::getPlatformState(const gz::sim::EntityComponentManager &ecm)
{
	const auto optional_pose = _link.WorldPose(ecm);

	if (optional_pose.has_value()) {
		_platform_position = optional_pose.value().Pos();
		_platform_orientation = optional_pose.value().Rot();

	} else {
		gzerr << "Unable to get pose" << std::endl;
	}

	const auto optional_vel = _link.WorldLinearVelocity(ecm);

	if (optional_vel.has_value()) {
		_platform_velocity = optional_vel.value();

	} else {
		gzerr << "Unable to get linear velocity" << std::endl;
	}

	const auto optional_angular_vel = _link.WorldAngularVelocity(ecm);

	if (optional_angular_vel.has_value()) {
		_platform_angular_velocity = optional_angular_vel.value();

	} else {
		gzerr << "Unable to get angular velocity" << std::endl;
	}
}

void BargeController::sendWrenchCommand(gz::sim::EntityComponentManager &ecm)
{
	_link.AddWorldWrench(ecm, _force, _torque);
}

void BargeController::getVehicleModelName()
{
	// Find the name of the gazebo vehicle model.
	// The name is constructed in px4-rc.gzsim as:
	//     MODEL_NAME="${PX4_SIM_MODEL#*gz_}"
	//     MODEL_NAME_INSTANCE="${MODEL_NAME}_${px4_instance}"
	// So here we replicate that.

	const char *px4_sim_model_cstr = std::getenv("PX4_SIM_MODEL");
	std::string px4_sim_model = "";

	const char *px4_gz_model_name_cstr = std::getenv("PX4_GZ_MODEL_NAME");

	if (px4_sim_model_cstr != nullptr) {

		px4_sim_model = px4_sim_model_cstr;

	} else if (px4_gz_model_name_cstr != nullptr) {

		// This happens if we attach to an existing model. In this case,
		// do not wait for any vehicle to spawn.

		gzwarn << "PX4_SIM_MODEL not set. Proceeding without vehicle name and moving platform immediately." << std::endl;
		_wait_for_vehicle_spawned = false;

	} else {

		// If neither are set, the px4-rc.gzsim script should have
		// exited 1. We could land here if these environment variables
		// change -- if so, update this function accordingly.

		gzerr << "Neither PX4_MODEL nor PX4_GZ_MODEL_NAME are set. One needed to proceed." << std::endl;
		_wait_for_vehicle_spawned = false;
	}

	// Remove leading "gz_"
	const std::string prefix_to_remove = "gz_";
	size_t pos = px4_sim_model.find(prefix_to_remove);

	if (pos != std::string::npos) {
		px4_sim_model = px4_sim_model.substr(pos + prefix_to_remove.length());

	} else {
		gzwarn << "Error: \"gz_\" not found in PX4_SIM_MODEL. Using the entire string as MODEL_NAME." << std::endl;
	}

	// Get the px4_instance environment variable
	const char *px4_instance_cstr = std::getenv("px4_instance");
	std::string px4_instance = "";

	if (px4_instance_cstr != nullptr) {
		px4_instance = px4_instance_cstr;

	} else {
		px4_instance = "0";
	}

	_vehicle_model_name = px4_sim_model + "_" + px4_instance;
	_wait_for_vehicle_spawned = true;
}

void BargeController::bargeCallback(const gz::msgs::Odometry &_msg)
{
	// Extract vehicle position and orientation
    // Assign position
    const auto pos_msg = _msg.pose().position();
	_position_sp.Set(pos_msg.x(), pos_msg.y(), pos_msg.z());

    // Assign orientation
    const auto &ori_msg = _msg.pose().orientation();
	_orientation_sp.Set(ori_msg.w(), ori_msg.x(), ori_msg.y(), ori_msg.z());

}
