from arucoquad import ArucoQuad, Tag
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from custom_msgs.msg import TagCorners, ControllerState, ControllerPoints, TargetOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum
import numpy as np
import px4_msgs.msg
import random
import rclpy
import sys


class st(Enum):
    OFFBOARD_ARM = 0
    SEARCH = 1
    APPROACH_CLOSE = 2
    APPROACH_FF = 3
    LANDING = 4
    LANDED = 5
    HEADING_CORRECTION = 6
    GOTO = 7
    LOITER = 8

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
    ARMED_RANDOM_START = 14

class X500(ArucoQuad):

    def __init__(self):

        # Init. functionality of various subclasses
        super().__init__('IBVS_node')

        # Search height to begin with
        self.declare_parameter('search_height', -20.0)
        self.search_height = self.get_parameter('search_height').get_parameter_value().double_value

        # Heading correction
        self.declare_parameter('heading_correction_gain', 0.65)
        self.heading_correction_gain = self.get_parameter('heading_correction_gain').get_parameter_value().double_value
        self.est_yaw_err = -1.0

        # Confidence required for IBVS
        self.declare_parameter('search_otag_confidence_threshold', 0.95)
        self.search_otag_confidence_threshold = self.get_parameter('search_otag_confidence_threshold').get_parameter_value().double_value

        # IBVS offset parameters
        self.declare_parameter('otag_offset', 125)
        self.declare_parameter('itag_offset', 115)
        self.declare_parameter('landing_offset', 200)

        self.otag_offset = self.get_parameter('otag_offset').get_parameter_value().integer_value
        self.itag_offset = self.get_parameter('itag_offset').get_parameter_value().integer_value
        self.landing_offset = self.get_parameter('landing_offset').get_parameter_value().integer_value

        # IBVS gain matrices
        self.declare_parameter('ogain', [.12, .12, .60, .40])
        self.declare_parameter('igain', [.20, .20, .20, .40])
        self.declare_parameter('lgain', [.30, .30, 1.0, .20])

        self.declare_parameter('use_ekf_z', True)
        self.use_ekf_z = self.get_parameter('use_ekf_z').get_parameter_value().bool_value
        self.declare_parameter('ekf_ff_enabled', True)
        self.ekf_ff_enabled = self.get_parameter('ekf_ff_enabled').get_parameter_value().bool_value

        self.ogain = self.get_parameter('ogain').get_parameter_value().double_array_value
        self.igain = self.get_parameter('igain').get_parameter_value().double_array_value
        self.lgain = self.get_parameter('lgain').get_parameter_value().double_array_value

        self.get_logger().info("ogain: " + str(self.ogain))
        self.get_logger().info("igain: " + str(self.igain))
        self.get_logger().info("lgain: " + str(self.lgain))
        
        self.declare_parameter('ibvs_z_approach_ff', 15.5)
        self.declare_parameter('ibvs_z_approach_close', 2.5)
        self.declare_parameter('ibvs_z_landing', 0.80)

        self.ibvs_z_approach_ff = self.get_parameter('ibvs_z_approach_ff').get_parameter_value().double_value
        self.ibvs_z_approach_close = self.get_parameter('ibvs_z_approach_close').get_parameter_value().double_value
        self.ibvs_z_landing = self.get_parameter('ibvs_z_landing').get_parameter_value().double_value

        self.declare_parameter('itag_start_landing_err', 125.0) # float is important
        self.itag_start_landing_err = self.get_parameter('itag_start_landing_err').get_parameter_value().double_value

        # Adaptive z gain variables
        self.declare_parameter('ad_z_k', 0.01)
        self.k = self.get_parameter('ad_z_k').get_parameter_value().double_value
        self.ad_z = 0.0 # Cho et al. 2022 adaptive gain variable

        # Mission simulation parameters
        self.declare_parameter('random_start', False)
        self.declare_parameter('simulate_missions', True)
        self.declare_parameter('max_missions', 5)
        self.declare_parameter('land_time', 20)
        self.declare_parameter('loiter_time', 10)

        self.random_start = self.get_parameter('random_start').get_parameter_value().bool_value
        self.simulate_missions = self.get_parameter('simulate_missions').get_parameter_value().bool_value
        self.max_missions = self.get_parameter('max_missions').get_parameter_value().integer_value
        self.land_time = self.get_parameter('land_time').get_parameter_value().integer_value
        self.loiter_time = self.get_parameter('loiter_time').get_parameter_value().integer_value
        
        # Create state machine states
        self.states = {
            st.SEARCH: self.t_search_state,
            st.HEADING_CORRECTION: self.t_heading_correction_state,
            st.APPROACH_CLOSE: self.t_approach_close_state,
            st.APPROACH_FF: self.t_approach_ff_state,
            st.LANDED: self.t_landed_state,
            st.OFFBOARD_ARM: self.t_offboard_arm_state,
            st.LANDING: self.t_landing_state,
            st.GOTO: self.t_goto_state,
            st.LOITER: self.t_loiter_state
        }
        self.current_state = st.OFFBOARD_ARM

        # Start up check variables
        self.offboard_enabled = False
        self.callback_test = False
        self.ekf_test = False

        # Start point coordinate
        self.start_pos = None

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
        self.goal_points = [Point(), Point(), Point(), Point()]

        # Blend variables
        self.otag_blend_factor = 0.0
        self.itag_blend_factor = 0.0

        # State timer
        self.state_time = None

        # Code for simulating autonomous missions
        self.simulate_missions = True
        self.land_time = 20 # Seconds

        self.missions_finished = 0
        self.max_missions = 5

        self.mission_generated = False
        self.loiter_time = random.randrange(10, 30)
        self.min_rad = 30
        self.max_rad = 120
        self.target_x = 0.0
        self.target_y = 0.0

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

        self.point_publisher = self.create_publisher(
            ControllerPoints, "x500/diag/ibvs_points", QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            ) # QoS settings for high reliability
        )

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
    
    # region state_transitions
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

        if event == ev.ARMED:
            self.get_logger().info("Successfully armed and offboarded the UAV: ARM_OFFBOARD -> SEARCH")
            self.state_time = self.get_clock().now().nanoseconds / 1e9
            self.current_state = st.SEARCH

        if event == ev.ARMED_RANDOM_START:
            self.get_logger().info("Successfully armed and offboarded the UAV, random mission start position: ARM_OFFBOARD -> GOTO")
            self.state_time = self.get_clock().now().nanoseconds / 1e9
            self.current_state = st.GOTO

    def t_search_state(self, event):

        if event == ev.OUT_TARGET_LOCKED:
            self.state_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info("Outer target lock acquired: SEARCH -> HEADING_CORRECTION")
            self.current_state = st.HEADING_CORRECTION
        
        else:
            self.transition_error(event)

    def t_heading_correction_state(self, event):

        if event == ev.OUT_TARGET_LOST:
            self.get_logger().info("Outer target lock lost: HEADING_CORRECTION -> SEARCH")
            self.state_time = self.get_clock().now().nanoseconds / 1e9
            self.current_state = st.SEARCH

        if event == ev.HEADING_CORRECTED:
            self.get_logger().info("Heading corrected: HEADING_CORRECTION -> APPROACH_FF")
            self.state_time = self.get_clock().now().nanoseconds / 1e9
            self.current_state = st.APPROACH_FF

        else:
            self.transition_error(event)

    def t_approach_ff_state(self, event):

        if event == ev.OUT_TARGET_LOST:
            self.get_logger().info("Inner target lock lost: CLOSE_APPROACH -> SEARCH")
            self.state_time = self.get_clock().now().nanoseconds / 1e9
            self.current_state = st.SEARCH

        if event == ev.IN_TARGET_LOCKED:
            self.get_logger().info("Centered on inner tag: APPROACH_FF -> APPROACH_CLOSE")
            self.state_time = self.get_clock().now().nanoseconds / 1e9
            self.current_state = st.APPROACH_CLOSE

    def t_approach_close_state(self, event):
        
        if event == ev.IN_TARGET_LOST:
            self.get_logger().info("Inner target lock lost: CLOSE_APPROACH -> SEARCH")
            self.state_time = self.get_clock().now().nanoseconds / 1e9
            self.current_state = st.SEARCH

        if event == ev.SENDING_IT:
            self.get_logger().info("Inner target stable, sending it: CLOSE_APPROACH -> LANDING")
            self.state_time = self.get_clock().now().nanoseconds / 1e9
            self.current_state = st.LANDING

        else:
            self.transition_error(event)

    def t_landing_state(self, event):

        if event == ev.LAND_DETC:
            self.get_logger().info("Land detected: LANDING -> LANDED")
            self.state_time = self.get_clock().now().nanoseconds / 1e9
            self.current_state = st.LANDED

        else:
            self.transition_error(event)

    def t_landed_state(self, event):

        if event == ev.START_MISSION:
            self.get_logger().info("Starting new mission: LANDED -> GOTO")
            self.state_time = self.get_clock().now().nanoseconds / 1e9
            self.current_state = st.GOTO
        
        else:
            self.transition_error(event)

    def t_goto_state(self, event):
        
        if event == ev.START_LOITER:
            self.get_logger().info("Starting loiter: GOTO -> LOITER")
            self.state_time = self.get_clock().now().nanoseconds / 1e9
            self.current_state = st.LOITER

        else:
            self.transition_error(event)

    def t_loiter_state(self, event):

        if event == ev.RETURN:
            self.get_logger().info("Returning to search: LOITER -> SEARCH")
            self.state_time = self.get_clock().now().nanoseconds / 1e9
            self.current_state = st.SEARCH

        else: 
            self.transition_error(event)

    #endregion

    def aruco_callback(self, msg: TagCorners):

        # Update tag confidences
        self.otag.update_confidence(msg.otag)
        self.itag.update_confidence(msg.itag)

        # Only run if tag detected:
        if msg.otag:
            self.otag.tracked = True

            # Update otag raw points
            self.otag.p = [msg.otag_corners[0], msg.otag_corners[1], msg.otag_corners[2], msg.otag_corners[3]]
            
            # Update otag virtual points
            self.otag.pv = self.map_to_virtual(self.otag.p)

        else:
            self.otag.tracked = False

        if msg.itag:
            self.itag.tracked = True

            # Update itag raw points
            self.itag.p = [msg.itag_corners[0], msg.itag_corners[1], msg.itag_corners[2], msg.itag_corners[3]]

            # Update itag virtual points
            self.itag.pv = self.map_to_virtual(self.itag.p)

        else:
            self.itag.tracked = False
        
    def ekf_callback(self, msg: TargetOdometry):
        self.target_ekf_odom = msg
        
        # EKF check
        if not self.ekf_test:
            self.ekf_test = True
    
    def state_timer_callback(self):

        if self.current_state in [st.APPROACH_FF, st.APPROACH_CLOSE, st.LANDING]:
            self.publish_offboard_control_heartbeat(velocity = True)
        else:
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

                # Wait for camera parameters to be populated
                if not self.camera_info_received:
                    self.get_logger().info("Waiting for camera info...", throttle_duration_sec=1.0)

                # Wait for callbacks to have fired at least once...
                if self.callback_test and self.camera_info_received:

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

                            # Log if EKF enabled during offbaord arm state
                            if (self.ekf_test):
                                self.get_logger().info("EKF active received during OFFBOARD_ARM", once=True)
                            else:
                                self.get_logger().warning("EKF active not received during OFFBOARD_ARM", once=True)

                            self.get_logger().info("PX4 in offboard mode and quadrotor armed", once=True)

                            if not self.random_start:
                                self.transition(ev.ARMED)
                            else:
                                self.transition(ev.ARMED_RANDOM_START)


            case st.SEARCH:

                if self.start_pos == None:
                    self.start_pos = self.get_vehicle_local_position()

                cmd_x, cmd_y, cmd_z = self.get_tracking_setpoint(); 

                if (self.get_vehicle_local_position().z > (self.search_height / 2)):
                    self.publish_velocity_setpoint(vx=0.0, vy=0.0, vz=-2.5, vyaw=0.0)
                else:
                    self.publish_position_setpoint(cmd_x, cmd_y, cmd_z, yaw=0.0)

                if self.otag.is_confident(self.search_otag_confidence_threshold):
                    self.get_logger().info(f"Outer tag confidence > {self.search_otag_confidence_threshold}: SEARCH -> HEADING_CORRECTION")
                    self.transition(ev.OUT_TARGET_LOCKED)


            case st.HEADING_CORRECTION:

                cmd_x, cmd_y, cmd_z = self.get_tracking_setpoint()

                # publish points for diag
                self.publish_points(tag="none")

                # Find and correct yaw value
                p1v = np.array([self.otag.pv[0].x, self.otag.pv[0].y])
                p4v = np.array([self.otag.pv[3].x, self.otag.pv[3].y])

                if (p4v[0] != p1v[0] and p4v[1] != p1v[1]):
                    v = (p4v - p1v) / np.linalg.norm(p4v - p1v)
                    theta_err = np.arctan2(v[1], v[0])

                else:
                    self.get_logger().info("p1v and p4v coordinates identical", throttle_duration_sec=1.0)
                    theta_err = 0.0

                self.est_yaw_err = theta_err
                # Simple p-controller to control heading
                heading_gain = self.heading_correction_gain
                yaw_cmd = heading_gain * theta_err

                self.publish_position_setpoint(cmd_x, cmd_y, cmd_z, yaw_speed=yaw_cmd)

                # Guard for lost outer tag lock
                if not self.otag.is_confident(.25):
                    self.get_logger().info("Lost tracking on outer tag: HEADING_CORRECTION -> SEARCH")
                    self.transition(ev.OUT_TARGET_LOST)

                # Transition to approach once heading is corrected and stable for a moment
                if (np.abs(theta_err) < np.deg2rad(3.0) and np.abs(self.get_vehicle_odometry().angular_velocity[2]) < 0.05):
                    self.get_logger().info(f"Theta_err on HEADING_CORRECTION -> APPROACH_FF: {theta_err:.2f}", throttle_duration_sec = 1)
                    self.transition(ev.HEADING_CORRECTED)


            case st.APPROACH_FF:
                
                # Calculate IBVS & update tag
                self.ibvs_cmd_raw, err = self.ibvs_wrapper(tag="otag", offset=self.otag_offset, gain_matrix=self.ogain)
                self.update_tag_error(err, self.otag)
                self.publish_points(tag="otag")

                # Blend in IBVS command to feed-forward slowly to avoid large kick
                now = self.get_clock().now().nanoseconds / 1e9
                elapsed = now - self.state_time # Find elapsed time

                # IBVS blend values
                blend_time = 2.0
                self.otag_blend_factor = min(1.0, elapsed/blend_time)
                self.ibvs_cmd = self.otag_blend_factor * self.ibvs_cmd_raw

                self.get_logger().info(f"Otag-IBVS blend: {self.otag_blend_factor:.2f}, ad_z: {self.ad_z:.2f}, elapsed: {elapsed:.2f}", throttle_duration_sec=0.1)
                
                if self.ekf_test and self.ekf_ff_enabled:
                    # Get EKF FF velocities
                    self.ff_cmd[0] = np.float32(self.target_ekf_odom.twist.twist.linear.x)
                    self.ff_cmd[1] = np.float32(self.target_ekf_odom.twist.twist.linear.y)

                    self.vel_cmd = self.ibvs_cmd + self.ff_cmd

                else:
                    # EKF not available
                    self.vel_cmd = self.ibvs_cmd
                    
                self.publish_velocity_setpoint(self.vel_cmd[0], self.vel_cmd[1], self.vel_cmd[2], self.vel_cmd[3])

                # Track whether to switch to landing case
                if self.itag.is_confident(.95) and elapsed > 0.0:
                    self.get_logger().info("Itag confident in APPROACH, switching IBVS targets")
                    self.transition(ev.IN_TARGET_LOCKED)

                # If not confidently locked on to the outer tag, abort approach and move to SEARCH state
                # Can be some pretty big swings here, so gotta be totally lost before changing back to search
                if not self.otag.is_confident(.05):
                    self.get_logger().info("Lost tracking on outer tag: APPROACH -> SEARCH")
                    self.transition(ev.OUT_TARGET_LOST)


            case st.APPROACH_CLOSE:

                # Calculate IBVS and update tag
                self.ibvs_cmd_raw, err = self.ibvs_wrapper(tag="itag", offset=self.itag_offset, gain_matrix=self.igain)
                self.update_tag_error(err, self.itag)
                self.publish_points(tag="itag")

                # Blend in IBVS command to feed-forward slowly to avoid large kick
                now = self.get_clock().now().nanoseconds / 1e9
                elapsed = now - self.state_time # Find elapsed time

                # IBVS blend values
                blend_time = 2.0
                self.itag_blend_factor = min(1.0, elapsed/blend_time)
                self.ibvs_cmd = self.itag_blend_factor * self.ibvs_cmd_raw

                self.get_logger().info(f"Itag blend: {self.itag_blend_factor:.2f}, ad_z: {self.ad_z:.2f}, itag_err/req: {self.itag.err_filtered:.2f}/{self.itag_start_landing_err}", throttle_duration_sec=0.1)

                if self.ekf_test and self.ekf_ff_enabled:
                    # Get EKF FF velocities
                    self.ff_cmd[0] = np.float32(self.target_ekf_odom.twist.twist.linear.x)
                    self.ff_cmd[1] = np.float32(self.target_ekf_odom.twist.twist.linear.y)

                    self.vel_cmd = self.ibvs_cmd + self.ff_cmd

                else:
                    # EKF not available
                    self.vel_cmd = self.ibvs_cmd

                self.publish_velocity_setpoint(self.vel_cmd[0], self.vel_cmd[1], self.vel_cmd[2], self.vel_cmd[3])
                
                # If not confidently locked on to the outer tag, abort approach and move to SEARCH state
                if not self.itag.is_confident(.20):
                    self.get_logger().info("Lost tracking on inner tag: APPROACH_CLOSE -> SEARCH")
                    self.transition(ev.IN_TARGET_LOST)            

                # If confident and near center of landing pad, land!
                if self.itag.err_filtered < self.itag_start_landing_err and self.itag.is_confident(0.9) and elapsed > 0.0:
                    self.get_logger().info("Itag centered and within error, sending it!")
                    self.transition(ev.SENDING_IT)


            case st.LANDING:

                if self.state_time == None:
                    self.state_time = self.get_clock().now().nanoseconds / 10e8
                    
                self.ibvs_cmd, _ = self.ibvs_wrapper(tag="itag", offset=self.landing_offset, gain_matrix=self.lgain)
                self.publish_points(tag="itag")

                if self.ekf_test and self.ekf_ff_enabled:
                    # Get EKF FF velocities
                    self.ff_cmd[0] = 1.01 * np.float32(self.target_ekf_odom.twist.twist.linear.x)
                    self.ff_cmd[1] = 1.01 * np.float32(self.target_ekf_odom.twist.twist.linear.y)

                    self.vel_cmd = self.ibvs_cmd + self.ff_cmd
                else:
                    self.vel_cmd = self.ibvs_cmd

                landing_velocity = 0.8
                self.vel_cmd[2] = landing_velocity # Overwrite landing velocity with just go down

                self.publish_velocity_setpoint(self.vel_cmd[0], self.vel_cmd[1], self.vel_cmd[2], self.vel_cmd[3])

                if (self.get_vehicle_land_detected().maybe_landed == True or self.get_vehicle_land_detected().landed == True):
                    self.get_logger().info("Land detected!")
                    self.missions_finished += 1
                    self.transition(ev.LAND_DETC)


            case st.LANDED:

                if self.simulate_missions and self.missions_finished < self.max_missions:
                    elapsed_time = self.get_clock().now().nanoseconds / 1e9 - self.state_time
                    self.get_logger().info(f"Landing elapsed time: {elapsed_time:.1f} / {self.land_time}, Missions: {self.missions_finished}/{self.max_missions}", throttle_duration_sec=1.0)

                    if (elapsed_time > self.land_time):
                        self.get_logger().info("Landing complete, simulating mission")
                        self.transition(ev.START_MISSION)

                if not self.simulate_missions or self.missions_finished <= self.max_missions:
                    self.get_logger().info("Completed landing, not starting more missions.")
                    sys.exit(0)


            case st.GOTO:

                if self.start_pos == None:
                    self.start_pos = self.get_vehicle_local_position()
                
                if not self.mission_generated:

                    rad = random.randrange(self.min_rad, self.max_rad)
                    angle = 2 * np.pi * random.random()
                    x = rad * np.cos(angle)
                    y = rad * np.sin(angle)

                    cur_x = self.get_vehicle_local_position().x
                    cur_y = self.get_vehicle_local_position().y

                    self.target_x = cur_x + x
                    self.target_y = cur_y + y
                    self.mission_generated = True

                # Offboard already done, but rearm needed
                if self.get_vehicle_status().arming_state != px4_msgs.msg.VehicleStatus.ARMING_STATE_ARMED:
                    self.get_logger().info("Arming quadrotor for mission", throttle_duration_sec=10.0)
                    self.arm(log=False)

                # Go to target coordinates if armed
                if self.get_vehicle_status().arming_state == px4_msgs.msg.VehicleStatus.ARMING_STATE_ARMED:
                    self.get_logger().info(f"Going to target: ({self.target_x:.1f}, {self.target_y:.1f})", throttle_duration_sec=1.0)
                    self.publish_position_setpoint(self.target_x, self.target_y, self.search_height)

                # Check distance to target and transition to loiter if reached
                distance_to_target = np.sqrt((self.target_x - self.get_vehicle_local_position().x)**2 + (self.target_y - self.get_vehicle_local_position().y)**2)
                self.get_logger().info(f"Distance to target: {distance_to_target:.1f}", throttle_duration_sec=0.5)

                if (distance_to_target < 1.0):
                    self.get_logger().info("Reached target position, loitering...")
                    self.transition(ev.START_LOITER)


            case st.LOITER:
                
                elapsed_time = self.get_clock().now().nanoseconds / 1e9 - self.state_time
                self.get_logger().info(f"Loiter elapsed time: {elapsed_time:.1f} / {self.loiter_time}", throttle_duration_sec=1.0)

                if (elapsed_time > self.loiter_time):
                    self.mission_generated = False
                    self.transition(ev.RETURN)

    def get_tracking_setpoint(self):
        """
        Function to simplify state machine logic during SEARCH and HEADING_CORRECTION states,
        where the drone is either tracking the EKF target or hovering at the start position if
        no EKF is available.
        """
        # Generate position setpoint
        if self.ekf_test and isinstance(self.start_pos, px4_msgs.msg.VehicleLocalPosition) and self.ekf_ff_enabled:
            # EKF tracks GPS coordinates while drone is not in line of sight
            tag_x = self.target_ekf_odom.pose.pose.position.x
            tag_y = self.target_ekf_odom.pose.pose.position.y

            # Low quality feedforward
            tag_vx = self.target_ekf_odom.twist.twist.linear.x
            tag_vy = self.target_ekf_odom.twist.twist.linear.y

            cmd_x = tag_x + 0.5 * tag_vx
            cmd_y = tag_y + 0.5 * tag_vy
            cmd_z = self.start_pos.z + self.search_height

        elif isinstance(self.start_pos, px4_msgs.msg.VehicleLocalPosition) and self.ekf_ff_enabled:
            # If doing test without EKF/without tag GPS + have pos
            cmd_x = self.start_pos.x
            cmd_y = self.start_pos.y
            cmd_z = self.start_pos.z + self.search_height

        else:
            cmd_x = 0
            cmd_y = 0
            cmd_z = self.search_height

        return cmd_x, cmd_y, cmd_z

    def ibvs_wrapper(self, tag, offset, gain_matrix):
        """
        Wrapper function to call IBVS solve function and convert output velocity command 
        to be in the frame of the vehicle, rather than the camera, as well as to return 
        error for tag tracking diagnostics.
        """

        if tag == "otag":
            points = self.otag.pv

        elif tag == "itag":
            points = self.itag.pv

        # Implement adaptive gain matrix based on image-plane distance to target from Cho et al. 2022
        target_center = np.array([
            np.mean([p.x for p in points]),
            np.mean([p.y for p in points])
            ],
            dtype=np.float32
        )
        c = np.linalg.norm(self.img_centre - target_center) # 'c' from paper
        self.ad_z = 2 * (1 - (1/(1 + np.exp(-self.k*c))))
        
        match self.current_state:
            case st.APPROACH_FF:
                z = self.ibvs_z_approach_ff
            case st.APPROACH_CLOSE:
                z = self.ibvs_z_approach_close
            case st.LANDING:
                z = self.ibvs_z_landing

        if self.use_ekf_z and self.ekf_test:
            # z = self.target_ekf_odom.pose.pose.position.z - self.get_vehicle_local_position().z
            z = self.target_ekf_odom.pose.pose.position.z # Use position directly (mapping creates issues)

        vel_cmd, err, goal_points, _ = self.solve_ibvs(p_arr=points, offset=offset, Z=z, gain=gain_matrix, ad_z=self.ad_z)

        heading = self.get_vehicle_local_position().heading

        # 2D map
        R = np.array([[np.cos(heading), -np.sin(heading)],
                [np.sin(heading),  np.cos(heading)]], dtype=np.float32)
                
        v_p = R @ np.array([[vel_cmd[0]], 
                            [vel_cmd[1]]])

        # Output vector: vx, vy, vz, vyaw
        v = np.array([v_p[0][0], v_p[1][0], vel_cmd[2], vel_cmd[3]], dtype=np.float32)

        # Publish ibvs point data
        self.goal_points = goal_points

        return v, err
        
    def publish_points(self, tag="none"):
        # points message
        point_msg = ControllerPoints()
        point_msg.header.stamp  = self.get_clock().now().to_msg()

        # Populate if visual servoing
        if tag == "otag":
            point_msg.otag = True
        elif tag == "itag":
            point_msg.itag = True
        
        # Point locations
        point_msg.otag_p = self.otag.p
        point_msg.otag_pv = self.otag.pv

        point_msg.itag_p = self.itag.p
        point_msg.itag_pv = self.itag.pv

        point_msg.goal_points = self.goal_points

        self.point_publisher.publish(point_msg)

    def update_tag_error(self, error, tag: Tag):
        """
        Updates filtered error on tags for each state machine step
        
        :param error: Vector of error from tag points to desired points
        """
        if tag.tracked:
            tag.update_err_filtered(error)
        else:
            tag.update_err_filtered(np.array([[300, 300], [300, 300]]))

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
        msg.ekf_ff = self.ekf_ff_enabled

        msg.itag_detc = self.itag.tracked
        msg.otag_detc = self.otag.tracked

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

        # Yaw estimation error
        if (self.current_state != st.HEADING_CORRECTION):
            msg.yaw_err = -1.0
        else:
            msg.yaw_err = float(self.est_yaw_err)

        # Floats
        msg.otag_conf = float(self.otag.confidence)
        msg.itag_conf = float(self.itag.confidence)
        msg.itag_err_filt = float(self.itag.err_filtered)
        msg.otag_err_filt = float(-1.0) # not used in current implementation

        msg.ad_z = float(self.ad_z)

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

        # Mission info
        msg.land_time = float(self.land_time)
        msg.loiter_time = float(self.loiter_time)
        msg.missions_finished = int(self.missions_finished)
        msg.max_missions = int(self.max_missions)

        msg.target_x = float(self.target_x)
        msg.target_y = float(self.target_y)

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
