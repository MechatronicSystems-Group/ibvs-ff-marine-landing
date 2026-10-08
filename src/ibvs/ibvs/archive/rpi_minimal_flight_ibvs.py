from arucoquad import ArucoQuad, Tag
from sensor_msgs.msg import CameraInfo
from custom_msgs.msg import TargetOdometry, ControllerState, TagCorners
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from collections import deque
from enum import Enum
import numpy as np
import px4_msgs.msg
import rclpy

class st(Enum):
    OFFBOARD_ARM = 0
    SEARCH = 1
    APPROACH = 2
    APPROACH_CLOSE = 3
    APPROACH_FF = 4
    LANDING = 5
    LANDED = 6
    HEADING_CORRECTION = 7
    GOTO = 8
    LOITER = 9

class ev(Enum):
    ARMED = 0
    OUT_TARGET_LOCKED = 1
    OUT_TARGET_LOST = 2
    IN_TARGET_LOCKED = 3
    IN_TARGET_LOST = 4
    LAND_DETC = 5
    SENDING_IT = 6
    FF_STABLE = 7
    IN_TARGET_CENTRE = 8
    START_FF = 9
    HEADING_CORRECTED = 10
    START_MISSION = 11
    START_LOITER = 12
    RETURN = 13

class X500(ArucoQuad):

    def __init__(self):
        # Init. functionality of various subclasses
        super().__init__()

        # Create state machine states
        self.states = {
            st.SEARCH: self.t_search_state,
            st.HEADING_CORRECTION: self.t_heading_correction_state,
            st.APPROACH_CLOSE: self.t_approach_close_state,
            st.APPROACH_FF: self.t_approach_ff_state,
            st.LANDED: self.t_landed_state,
            st.OFFBOARD_ARM: self.t_offboard_arm_state,
            st.LANDING: self.t_landing_state,
        }
        self.current_state = st.OFFBOARD_ARM

        # Search height to begin with
        self.search_height = -2.5

        # Start up variables
        self.offboard_enabled = False
        self.callback_test = False
        self.ekf_test = False

        # Time tracking variable
        self.start_time = self.get_clock().now()
        self.elapsed_time = 0
        self.last_time = None

        # Velocity tracking variables
        self.ibvs_cmd = np.zeros([4], dtype=np.float32)
        self.ff_cmd = np.zeros([4], dtype=np.float32)
        self.vel_cmd = np.zeros([4], dtype=np.float32)

        # IBVS objects, functions and booleans
        self.otag = Tag()
        self.itag = Tag()

        self.ogain = [.2, .2,  .4, .4] # IBVS outer tag gain
        self.igain = [.5, .5,  .6, .4] # IBVS inner tag gain
        self.lgain = [.6, .6, 1.0, .2] # IBVS inner tag landing gain

        self.otag_deque_length = 80
        self.otag_err_moving_avg = deque(maxlen=self.otag_deque_length)
        self.otag_err_moving_avg_threshold = 5.00
        self.search_otag_confidence_threshold = 0.95

        self.itag_start_landing_err = 85.0

        self.otag_offset = 135
        self.itag_offset = 115
        self.landing_offset = 200

        self.itag_err_moving_avg = deque(maxlen=self.otag_deque_length)
        
        # State timer
        self.state_time = None

        # Velocity estimator & feed-forward velocity variables
        self.ekf_ff = False

        # Landing variables
        self.landing_pvz = 0

        # Blend variables
        self.otag_blend_factor = 0.0
        self.itag_blend_factor = 0.0

        # Subscribe to ArucoDetcs
        self.aruco_detcs_subscriber = self.create_subscription(
            TagCorners, "/tag/corners", self.aruco_callback, QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT, 
                durability=DurabilityPolicy.VOLATILE, 
                history=HistoryPolicy.KEEP_ALL, 
                depth=1
            )
        )

        # Subscribe to camera info
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo, "/x500/camera/camera_info", self.camera_param_callback, QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT, 
                durability=DurabilityPolicy.VOLATILE, 
                history=HistoryPolicy.KEEP_ALL, 
                depth=1
            )
        )

        # Subscribe to EKF filter
        self.target_ekf_subscriber = self.create_subscription(
            TargetOdometry, "/x500/ekf/target_odometry", self.ekf_callback, self.qos_px4
        )
        self.target_ekf_odom = None

        # Diagnostics publisher(s)
        self.state_publisher = self.create_publisher(
            ControllerState, "/x500/diag/state", QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            ) # QoS settings for high reliability
        )
        self.publish_state()

        # State timer
        self.state_machine_hz = 50
        self.state_timer = self.create_timer(1/self.state_machine_hz, self.state_timer_callback)

    def get_time(self):
        """
        Function to get the current time, which also returns the previous time for use in conditional info logging
        Outputs time and prev_time in seconds, not nanoseconds
        """
        self.prev_time = self.elapsed_time
        self.elapsed_time = int((self.get_clock().now() - self.start_time).nanoseconds / 1e9)

        return self.elapsed_time, self.prev_time
    
    def transition(self, event):
        """
        Transitions out of a state based on a passed event, where the reaction to the event is 
        handled by a dedicated 't_' function to handle different events that can plausibly happen while in a given state.
        As defined in self.states each system state has to have an associated event handling function - with given condition(s)
        to transit from itself to different state by setting self.current_state
        """
        self.states[self.current_state](event)

    def transition_error(self, event):
        """
        Default function to trigger if state transition function given event for which it has ill-defined behavior.
        """
        self.get_logger().error(f"State {str(self.current_state)} given unexpected event: {str(event)}")

    def t_offboard_arm_state(self, event):

        self.state_time = self.get_clock().now().nanoseconds / 1e9

        self.state_time = None

        if event == ev.ARMED:
            self.get_logger().info("Successfully armed and offboarded the UAV: ARM_OFFBOARD -> SEARCH")
            self.current_state = st.SEARCH

    def t_search_state(self, event):

        self.state_time = self.get_clock().now().nanoseconds / 1e9

        if event == ev.OUT_TARGET_LOCKED:
            self.get_logger().info("Outer target lock acquired: SEARCH -> HEADING_CORRECTION")
            self.current_state = st.HEADING_CORRECTION
        
        else:
            self.transition_error(event)

    def t_heading_correction_state(self, event):

        self.state_time = self.get_clock().now().nanoseconds / 1e9

        if event == ev.HEADING_CORRECTED:
            self.get_logger().info("Heading corrected: HEADING_CORRECTION -> APPROACH_FF")
            self.current_state = st.APPROACH_FF

        else:
            self.transition_error(event)

    def t_approach_ff_state(self, event):

        self.state_time = self.get_clock().now().nanoseconds / 1e9

        if event == ev.OUT_TARGET_LOST:
            self.get_logger().info("Inner target lock lost: CLOSE_APPROACH -> SEARCH")
            self.current_state = st.SEARCH

        if event == ev.IN_TARGET_LOCKED:
            self.get_logger().info("Centered on inner tag: APPROACH_FF -> APPROACH_CLOSE")
            self.current_state = st.APPROACH_CLOSE

    def t_approach_close_state(self, event):
        
        self.state_time = self.get_clock().now().nanoseconds / 1e9

        if event == ev.IN_TARGET_LOST:
            self.get_logger().info("Inner target lock lost: CLOSE_APPROACH -> SEARCH")
            self.current_state = st.SEARCH

        if event == ev.SENDING_IT:
            self.get_logger().info("Inner target stable, sending it: CLOSE_APPROACH -> LANDING")
            self.current_state = st.LANDING

        else:
            self.transition_error(event)

    def t_landing_state(self, event):

        self.state_time = self.get_clock().now().nanoseconds / 1e9

        if event == ev.LAND_DETC:
            self.get_logger().info("Land detected: LANDING -> LANDED")
            self.current_state = st.LANDED

        else:
            self.transition_error(event)

    def t_landed_state(self, event):

        self.state_time = self.get_clock().now().nanoseconds / 1e9

        # This should not receive any events (right now anyway)
        self.transition_error(event)

    def aruco_callback(self, msg: TagCorners):

        # Update tag confidences
        self.otag.update_confidence(msg.otag)
        self.itag.update_confidence(msg.itag)

        # Only run if tag detected:
        if msg.otag:
            self.otag.tracked = True

            # Update otag * raw points *
            self.otag.p = [msg.otag_corners[0], msg.otag_corners[1], msg.otag_corners[2], msg.otag_corners[3]]

            # Update otag * virtual points *
            self.otag.pv = self.map_to_virtual(self.otag.p)
            
        else:
            self.otag.tracked = False

        if msg.itag:
            self.itag.tracked = True

            # Update itag * raw points *
            self.itag.p = [msg.itag_corners[0], msg.itag_corners[1], msg.itag_corners[2], msg.itag_corners[3]]

            # Update itag * virtual points *
            self.itag.pv = self.map_to_virtual(self.itag.p)

        else:
            self.itag.tracked = False
        
    def ekf_callback(self, msg: TargetOdometry):
        self.target_ekf_odom = msg
    
    def state_timer_callback(self):

        # Get time in seconds
        time, prev_time = self.get_time()

        self.publish_offboard_control_heartbeat(position = True)
        self.publish_state()

        match self.current_state:


            case st.OFFBOARD_ARM:
                # Vehicle callback tests
                if self.get_vehicle_status() != None and self.get_vehicle_land_detected() != None and self.get_vehicle_local_position() != None and not self.callback_test:
                    self.get_logger().info("Vehicle callback check passed.")
                    self.callback_test = True

                else:
                    self.get_logger().info("Waiting for get_vehicle_status() to populate...", throttle_duration_sec=1.0)

                # Wait for EKF to have some idea whats going on
                if isinstance(self.target_ekf_odom, TargetOdometry) and not self.ekf_test:
                    self.get_logger().info("Barge EKF callback check passed.")
                    self.ekf_test = True

                else:
                    self.get_logger().info("Waiting for data from barge EKF...", throttle_duration_sec=1.0)

                # Wait for camera parameters to be populated
                if not self.camera_info_received:
                    self.get_logger().info("Waiting for camera info...", throttle_duration_sec=1.0)

                # Wait for callbacks to have fired at least once...
                if self.callback_test and self.ekf_test and self.camera_info_received:

                    # Very sequential offboard enable and arm sequence
                    if isinstance(self.get_vehicle_status(), px4_msgs.msg.VehicleStatus):
                        # Enable offboard mode if not yet enabled
                        if not self.offboard_enabled:
                            self.engage_offboard_mode()

                        # If it successfully enables we can move on with life
                        if (self.get_vehicle_status().nav_state == px4_msgs.msg.VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                            self.get_logger().info("Successfully entered offboard navigation state", once=True)
                            self.offboard_enabled = True

                        # If offboard enabled, arm
                        if self.offboard_enabled & (self.get_vehicle_status().arming_state != px4_msgs.msg.VehicleStatus.ARMING_STATE_ARMED):
                            self.get_logger().info("Arming quadrotor", once=True)
                            self.arm(log=False)

                        # Transition to search once offboard is established and quadrotor is armed
                        if self.get_vehicle_status().arming_state == px4_msgs.msg.VehicleStatus.ARMING_STATE_ARMED:
                            self.get_logger().info("PX4 in offboard mode and quadrotor armed", once=True)
                            self.transition(ev.ARMED)


            case st.SEARCH:

                # EKF tracks GPS coordinates while drone is not in line of sight
                tag_y = self.target_ekf_odom.pose.pose.position.y
                tag_x = self.target_ekf_odom.pose.pose.position.x
                tag_z = self.target_ekf_odom.pose.pose.position.z

                if (self.get_vehicle_local_position().z > -1.0):
                    #self.publish_position_setpoint(z=tag_z + self.search_height)
                    self.publish_velocity_setpoint(vx=0.0, vy=0.0, vz=-0.75, vyaw=0.0)
                else:
                    self.publish_position_setpoint(tag_x, tag_y, self.search_height)

                if self.otag.is_confident(self.search_otag_confidence_threshold):
                    self.get_logger().info(f"Outer tag confidence > {self.search_otag_confidence_threshold}: SEARCH -> HEADING_CORRECTION")
                    self.transition(ev.OUT_TARGET_LOCKED)


            case st.HEADING_CORRECTION:

                    self.get_logger().info("Skipping for now. Move straight to approach FF")
                    self.transition(ev.HEADING_CORRECTED)


            case st.APPROACH_FF:
                
                # New approach - start with FF and fade in aruco_tag
                if isinstance(self.target_ekf_odom, TargetOdometry) and not self.ekf_ff:
                    self.get_logger().info("Enabling EKF feedforward")
                    self.ekf_ff = True

                # Calculate IBVS & update tag
                self.ibvs_cmd_raw, err = self.ibvs_wrapper(points=self.otag.p, offset=self.otag_offset, z=2.5, gain_matrix=self.ogain)
                self.update_tag_error(err, self.otag)


                now = self.get_clock().now().nanoseconds / 1e9
                elapsed = now - self.state_time # Find elapsed time

                # IBVS blend values
                blend_time = 4
                self.otag_blend_factor = min(1, elapsed/blend_time)
                self.ibvs_cmd = self.otag_blend_factor * self.ibvs_cmd_raw

                if (self.otag_blend_factor < 1):
                    self.get_logger().info(f"Otag-IBVS blend: {self.otag_blend_factor:.2f}")
                
                if self.ekf_ff:
                    # Get EKF FF velocities
                    self.ff_cmd[0] = np.float32(self.target_ekf_odom.twist.twist.linear.x)
                    self.ff_cmd[1] = np.float32(self.target_ekf_odom.twist.twist.linear.y)

                    self.vel_cmd = self.ibvs_cmd + self.ff_cmd

                else:
                    # EKF not available or ready
                    self.vel_cmd = self.ibvs_cmd
                    
                self.publish_velocity_setpoint(self.vel_cmd[0], self.vel_cmd[1], self.vel_cmd[2], self.vel_cmd[3])
                
                # Track whether to switch to landing case

                if self.itag.is_confident(.95):
                    self.get_logger().info("Itag confident in APPROACH, switching IBVS targets")
                    self.transition(ev.IN_TARGET_LOCKED)

            case st.APPROACH_CLOSE:

                # Get EKF FF velocities
                self.ff_cmd[0] = np.float32(self.target_ekf_odom.twist.twist.linear.x)
                self.ff_cmd[1] = np.float32(self.target_ekf_odom.twist.twist.linear.y)

                # IBVS wrapper
                self.ibvs_cmd_raw, err = self.ibvs_wrapper(points=self.itag.pv, offset=self.itag_offset, z=1.5, gain_matrix=self.igain)
                self.update_tag_error(err, self.itag)

                # Blend in IBVS command to feed-forward slowly to avoid large kick
                # Can probably correspondingly update IBVS gains :^) 
                now = self.get_clock().now().nanoseconds / 1e9
                elapsed = now - self.state_time # Find elapsed time

                # IBVS blend values
                blend_time = 4
                self.itag_blend_factor = min(1, elapsed/blend_time)
                self.ibvs_cmd = self.itag_blend_factor * self.ibvs_cmd_raw

                if (self.itag_blend_factor < 1):
                    self.get_logger().info(f"Itag-IBVS blend: {self.itag_blend_factor:.2f}")

                self.vel_cmd = self.ibvs_cmd + self.ff_cmd
                self.publish_velocity_setpoint(self.vel_cmd[0], self.vel_cmd[1], self.vel_cmd[2], self.vel_cmd[3])
                
                # If not confidently locked on to the outer tag, abort approach and move to SEARCH state
                if not self.itag.is_confident(.20):
                    self.get_logger().info("Lost tracking on inner tag: APPROACH_CLOSE -> SEARCH")
                    self.transition(ev.IN_TARGET_LOST)

                # Log itag err
                self.get_logger().info(f"Filtered itag error: {self.itag.err_filtered:.2f}", throttle_duration_sec=1.0)                

                if self.itag.err_filtered < self.itag_start_landing_err and self.itag.is_confident(0.9):
                    self.get_logger().info("Itag centered and within error, sending it!")
                    self.transition(ev.SENDING_IT)


            case st.LANDING:

                if self.state_time == None:
                    self.state_time = self.get_clock().now().nanoseconds / 10e8
                    
                self.ibvs_cmd, _ = self.ibvs_wrapper(points=self.itag.pv, offset=self.landing_offset, z=1, gain_matrix=self.lgain)

                # Get EKF FF velocities
                self.ff_cmd[0] = np.float32(self.target_ekf_odom.twist.twist.linear.x)
                self.ff_cmd[1] = np.float32(self.target_ekf_odom.twist.twist.linear.y)

                self.vel_cmd = self.ibvs_cmd + self.ff_cmd

                landing_velocity = 0.8
                self.vel_cmd[2] = landing_velocity # Overwrite landing velocity with just go down

                self.publish_velocity_setpoint(self.vel_cmd[0], self.vel_cmd[1], self.vel_cmd[2], self.vel_cmd[3])

                if (self.get_vehicle_land_detected().maybe_landed == True or self.get_vehicle_land_detected().landed == True):
                    self.get_logger().info("Land detected!")
                    self.transition(ev.LAND_DETC)


            case st.LANDED:
                self.get_logger().info("Someone pour gatorade on Conrad, we got 'em!!", once=True)
                pass


    def ibvs_wrapper(self, points, offset, z, gain_matrix):
        vel_cmd, err, _, _ = self.solve_ibvs(p_arr=points, offset=offset, Z=z, gain=gain_matrix)

        heading = self.get_vehicle_local_position().heading

        # 2D map
        R = np.array([[np.cos(heading), -np.sin(heading)],
                [np.sin(heading),  np.cos(heading)]], dtype=np.float32)
                
        v_p = R @ np.array([[vel_cmd[0]], 
                            [vel_cmd[1]]])

        # Output vector: vx, vy, vz, vyaw
        v = np.array([v_p[0][0], v_p[1][0], vel_cmd[2], vel_cmd[3]], dtype=np.float32)

        return v, err
        
    def update_tag_error(self, error, tag: Tag):
        """
        Updates filtered error on tags for each state machine step
        
        :param error: Vector of error from tag points to desired points
        """
        if tag.tracked:
            tag.update_err_filtered(error)
        else:
            tag.update_err_filtered(np.array([[300, 300], [300, 300]]))
        
        if tag == self.otag:
            self.otag_err_moving_avg.append(self.otag.err_filtered)
        else:
            self.itag_err_moving_avg.append(self.itag.err_filtered)

    def publish_state(self):
        """
        Simple function that creates and publishes a message containing
        the current state of the IBVS-FF state-machine.
        """

        msg = ControllerState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.current_state.name

        # Booleans
        msg.offboard_enabled = self.offboard_enabled
        msg.callback_test = self.callback_test
        msg.ekf_test = self.ekf_test
        msg.ekf_ff = self.ekf_ff

        # IBVS state
        msg.ibvs = True if (self.current_state in [st.APPROACH_FF, st.APPROACH_CLOSE, st.LANDING]) else False

        # Blend factor
        msg.otag_factor = float(self.otag_blend_factor) if (self.current_state == st.APPROACH_FF) else 0.0
        msg.itag_factor = float(self.itag_blend_factor) if (self.current_state == st.APPROACH_CLOSE) else 0.0

        # Goal offset
        match self.current_state:
            case st.APPROACH_FF:
                msg.goal_offset = self.otag_offset
            case st.APPROACH_CLOSE:
                msg.goal_offset = self.itag_offset
            case st.LANDING:
                msg.goal_offset = self.landing_offset
            case _:
                msg.goal_offset = -1 # invalid value (sort of)

        # Floats
        msg.otag_conf = float(self.otag.confidence)
        msg.itag_conf = float(self.itag.confidence)
        msg.otag_err_filt = float(self.otag.err_filtered)
        msg.itag_err_filt = float(self.itag.err_filtered)

        # Arrays
        msg.ibvs_v = self.ibvs_cmd
        msg.ff_v = self.ff_cmd
        msg.cmd_v = self.vel_cmd

        # Publish local coordinates
        if self.get_vehicle_local_position() != None:
            msg.px4_x = self.get_vehicle_local_position().x
            msg.px4_y = self.get_vehicle_local_position().y
            msg.px4_z = self.get_vehicle_local_position().z
        else:
            msg.px4_x = np.nan
            msg.px4_y = np.nan
            msg.px4_z = np.nan

        self.state_publisher.publish(msg)

def main(args=None) -> None:

    # Spin up ROS2
    rclpy.init(args=args)

    x500_rpi_test_node = X500()

    try:
        rclpy.spin(x500_rpi_test_node)
    except SystemExit:
        x500_rpi_test_node.destroy_node()

if __name__ == '__main__':
    main()
