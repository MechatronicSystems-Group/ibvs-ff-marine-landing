from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLandDetected, VehicleAttitude, VehicleLocalPosition, VehicleOdometry, VehicleAttitudeSetpoint, VehicleRatesSetpoint, GotoSetpoint, HomePosition, VehicleGlobalPosition
import numpy as np
import platform

class Quad(Node):
    """ 
    Underlay functionality for controlling a drone in offboard, without having to define all these functions in the main state machine
    Current functions:
    """

    def __init__(self, node_name = "quad") -> None:
        print("Low level quad name : " + node_name)
        super().__init__(node_name)

        # Configure QoS profile to interact with PX4 over ROS2 network
        self.qos_px4 = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.VOLATILE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1
        )

        # Create necessary publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", self.qos_px4
        )

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self.qos_px4
        )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", self.qos_px4
        )

        self.attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, "/fmu/in/vehicle_attitude_setpoint_v1", self.qos_px4
        )

        self.rates_setpoint_publisher = self.create_publisher(
            VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", self.qos_px4
        )

        # Create necessary subscribers
        self.vehicle_status = None # In case get_vehicle_status is called before it is set by callback
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v1", self.vehicle_status_callback, self.qos_px4
        )

        self.vehicle_land_detected = None # In case get_vehicle_land_status is called before it is set by callback
        self.vehicle_land_subscriber = self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected", self.vehicle_land_detected_callback, self.qos_px4
        )

        self.vehicle_attitude = None
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, "fmu/out/vehicle_attitude", self.vehicle_attitude_callback, self.qos_px4
        )

        self.vehicle_local_position = None
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, "fmu/out/vehicle_local_position_v1", self.vehicle_local_position_callback, self.qos_px4 
        )
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, "fmu/out/vehicle_local_position", self.vehicle_local_position_callback, self.qos_px4 
        )

        self.vehicle_global_position = None
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, "fmu/out/vehicle_global_position", self.vehicle_global_position_callback, self.qos_px4
        )

        self.vehicle_odometry = None
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, "fmu/out/vehicle_odometry", self.vehicle_odometry_callback, self.qos_px4
        )

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        """
        Callback for receieved vehicle_status msg
        """
        self.vehicle_status = msg

    def get_vehicle_status(self) -> VehicleStatus:
        """
        Returns latest vehicle_status msg
        """
        return self.vehicle_status

    def vehicle_land_detected_callback(self, msg: VehicleLandDetected) -> None:
        """
        Callback for received vehicle_land_detected msg
        """
        self.vehicle_land_detected = msg

    def get_vehicle_land_detected(self) -> VehicleLandDetected:
        """
        Returns latest vehicle_land_detected msg
        """
        return self.vehicle_land_detected
    
    def vehicle_attitude_callback(self, msg: VehicleAttitude) -> None:
        """
        Callback for received vehicle_attitude msg
        """
        self.vehicle_attitude = msg

    def get_vehicle_attitude(self) -> VehicleAttitude:
        """
        Returns latest vehicle_attitude msg
        """
        return self.vehicle_attitude
    
    def vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        """
        Callback for received vehicle_odometry msg
        """
        self.vehicle_odometry = msg

    def get_vehicle_odometry(self) -> VehicleOdometry:
        """
        Returns latest vehicle_odometry msg
        """
        return self.vehicle_odometry
    
    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        """
        Callback for received vehicle_local_position msg
        """
        self.vehicle_local_position = msg

    def get_vehicle_local_position(self) -> VehicleLocalPosition:
        """
        Returns latest vehicle_local_position msg
        """
        return self.vehicle_local_position
    
    def vehicle_global_position_callback(self, msg: VehicleGlobalPosition) -> None:
        """
        Callback for recevied vehicle_global_position msg
        """
        self.vehicle_global_position = msg

    def get_vehicle_global_position(self) -> VehicleGlobalPosition:
        """
        Returns latest vehicle_global_position msg
        """
        return self.vehicle_global_position

    def publish_vehicle_command(self, command, **params) -> None:
        """
        Publish a VehicleCommand message to PX4
        """
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self, log=True) -> None:
        """
        Send an arm command to the vehicle
        """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        if log: self.get_logger().info('Arm command sent')

    def disarm(self) -> None:
        """
        Send a disarm command to the vehicle
        """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )
        # self.get_logger().info('Disarm command sent')

    def takeoff(self) -> None:
        """
        Send a takeoff command to the vehicle
        """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        )
        self.get_logger().info('Takeoff command sent')

    def land(self) -> None:
        """
        Send a land command
        """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )
        # self.get_logger().info('Land command sent')

    def engage_offboard_mode(self) -> None:
        """
            Starts offboard control mode on PX4. Only needs to run once, use in __init__ function of main class.
        """
        if isinstance(self.get_vehicle_status(), VehicleStatus):
            if (self.get_vehicle_status().nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self.publish_offboard_mode()

    def publish_offboard_mode(self) -> None:
        """
        Switch to offboard mode - offboard control type defined by publish_offboard_control_heartbeat()
        """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info('Switching to offboard control mode')

    def publish_offboard_control_heartbeat(self, **params) -> None:
        """
        Publish offboard control type -> velocity, position, attitude... etc, defined by params above
        """
        if any(params):
            msg = OffboardControlMode()
            msg.position     = params.get("position", False)
            msg.velocity     = params.get("velocity", False)
            msg.acceleration = params.get("acceleration", False)
            msg.attitude     = params.get("attitude", False)
            msg.body_rate    = params.get("body_rate", False)
            msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)

            # Publish completed msg
            self.offboard_control_mode_publisher.publish(msg)

        else:
            self.get_logger().error("Offboard control heartbeat needs to have at least 1 true control mode passed as **params")

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0, yaw_speed: float = None) -> None:
        """
        Publish position setpoint in world-frame coordinates. Note yaw_speed will overwrite yaw if present.
        """
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]

        if (yaw_speed != None):
            msg.yaw = np.nan
            msg.yawspeed = yaw_speed

        else:
            msg.yaw = yaw

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float, vyaw: float) -> None:
        """
        Publish velocity setpoint in body-frame coordinates (I _think_)
        """
        msg = TrajectorySetpoint()
        # Make sure that pos data is not zero but nan to be ignored by flight controller
        msg.position = np.array([np.nan, np.nan, np.nan]).astype(np.float32)
        msg.yaw = float(np.nan)

        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.yawspeed = float(vyaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_attitude_setpoint(self, q, thrust, yaw_rate) -> None:
        """
        Publish attitude setpoint
        """
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.yaw_sp_move_rate = yaw_rate
        msg.q_d = q
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = -np.clip(thrust, a_min=0.1, a_max=1.0)

        self.attitude_setpoint_publisher.publish(msg)

    def publish_rates_setpoint(self, vroll, vpitch, vyaw, thrust) -> None:
        """
        Publish rates setpoint
        """
        msg = VehicleRatesSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.roll = vroll
        msg.pitch = vpitch
        msg.yaw = vyaw

        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = -np.clip(thrust, a_min=0.1, a_max=1.0)

        self.rates_setpoint_publisher.publish(msg)

    def get_device_type(self):
        machine = platform.machine().lower()
        
        if machine == 'x86_64':
            return "desktop"
        elif machine.startswith('arm') or machine.startswith('aarch64'):
            return "raspberry_pi"
        else:
            return f"unknown_architecture_{machine}"