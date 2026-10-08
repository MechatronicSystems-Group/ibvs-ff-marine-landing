from ibvs.quad import ArucoQuad, Tag
from custom_msgs.msg import ArucoDetcs
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum
import numpy as np
import px4_msgs.msg
import px4_msgs
import rclpy

class st(Enum):
    OFFBOARD_ARM = 0
    SEARCH = 1
    APPROACH = 2
    FINE_APPROACH = 3
    LANDING = 4
    LANDED = 5

class ev(Enum):
    ARMED = 0
    OUT_TARGET_LOCKED = 1
    OUT_TARGET_LOST = 2
    IN_TARGET_LOCKED = 3
    IN_TARGET_LOST = 4
    LAND_DETC = 5
    SENDING_IT = 6

class X500(ArucoQuad):

    def __init__(self):
        # Init. functionality of various subclasses
        super().__init__()

        # Create state machine states
        self.states = {
            st.SEARCH: self.t_search_state,
            st.APPROACH: self.t_approach_state,
            st.FINE_APPROACH: self.t_fine_approach_state,
            st.LANDED: self.t_landed_state,
            st.OFFBOARD_ARM: self.t_offboard_arm_state,
            st.LANDING: self.t_landing_state
        }
        self.current_state = st.OFFBOARD_ARM

        # Search height to begin with
        self.search_height = -5

        # Start up variables
        self.offboard_enabled = False
        self.callback_test = False

        # Time tracking variable
        self.start_time = self.get_clock().now()
        self.elapsed_time = 0
        self.last_time = None

        # IBVS objects, functions and booleans
        self.otag = Tag()
        self.itag = Tag()

        self.ogain = [0.3, 0.3, 0.3, 0.2] # IBVS outer tag gain
        self.igain = [0.6, 0.6, 0.08, 0.2] # IBVS inner tag gain
        self.lgain = [0.3, 0.3, 1, 0.2] # IBVS inner tag gain
        self.set_camera_parameters() # Set camera parameters

        self.otag_err = 120
        self.itag_err = 80

        self.landing_pvz = 0

        # Subscribe to ArucoDetcs
        self.aruco_detcs_subscriber = self.create_subscription(
            ArucoDetcs, "/aruco_detcs", self.aruco_callback, QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT, 
                durability=DurabilityPolicy.VOLATILE, 
                history=HistoryPolicy.KEEP_ALL, 
                depth=1
            )
        )

        # Subscribe to Lidar z-filter
        self.zfilter_subscriber = self.create_subscription(
            Odometry, "/x500/lidar_filter", self.zfilter_callback, QoSProfile(
                reliability = ReliabilityPolicy.BEST_EFFORT,
                durability = DurabilityPolicy.VOLATILE,
                history = HistoryPolicy.KEEP_LAST, 
                depth = 1
            )
        )
        self.z = 2 # Initial guess
        self.z_vel = 0

        # State timer
        self.state_timer = self.create_timer(0.02, self.state_timer_callback)

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
        self.error(f"State {str(self.current_state)} given unexpected event: {str(event)}")

    def t_offboard_arm_state(self, event):

        if event == ev.ARMED:
            self.info("Successfully armed and offboarded the UAV: ARM_OFFBOARD -> SEARCH")
            self.current_state = st.SEARCH

    def t_search_state(self, event):

        if event == ev.OUT_TARGET_LOCKED:
            self.info("Outer target lock acquired: SEARCH -> APPROACH")
            self.current_state = st.APPROACH
        
        else:
            self.transition_error(event)

    def t_approach_state(self, event):

        if event == ev.OUT_TARGET_LOST:
            self.info("Outer target lock lost: APPROACH -> SEARCH")
            self.current_state = st.SEARCH

        if event == ev.IN_TARGET_LOCKED:
            self.info("Inner target lock acquired: APPROACH -> FINE_APPROACH")
            self.current_state = st.FINE_APPROACH

        else:
            self.transition_error(event)

    def t_fine_approach_state(self, event):

        if event == ev.IN_TARGET_LOST:
            self.info("Inner target lock lost: FINE_APPROACH -> SEARCH")
            self.current_state = st.SEARCH

        if event == ev.SENDING_IT:
            self.info("Inner target stable, sending it: FINE_APPROACH -> LANDING")
            self.current_state = st.LANDING

        else:
            self.transition_error(event)

    def t_landing_state(self, event):

        if event == ev.LAND_DETC:
            self.info("Land detected: LANDING -> LANDED")
            self.current_state = st.LANDED

        else:
            self.transition_error(event)

    def t_landed_state(self, event):
        # This should not receive any events (right now anyway)
        self.transition_error(event)

    def zfilter_callback(self, msg: Odometry):
        # Updates current a and z_vel estimate
        self.z = msg.pose.pose.position.z
        self.z_vel = msg.twist.twist.linear.z

    def aruco_callback(self, msg: ArucoDetcs):

        # Update tag confidences
        self.otag.update_confidence(msg.outer_tag_detc)
        self.itag.update_confidence(msg.inner_tag_detc)

        # Only run if tag detected:
        if msg.outer_tag_detc:
            self.otag.tracked = True

            # Update otag * raw points *
            self.otag.p = [msg.outer_tag.p1, msg.outer_tag.p2, msg.outer_tag.p3, msg.outer_tag.p4]

            # Update otag * virtual points *
            self.otag.pv = self.map_to_virtual(self.otag.p)

        else:
            self.otag.tracked = False

        if msg.inner_tag_detc:
            self.itag.tracked = True

            # Update itag * raw points *
            self.itag.p = [msg.inner_tag.p1, msg.inner_tag.p2, msg.inner_tag.p3, msg.inner_tag.p4]

            # Update itag * virtual points *
            self.itag.pv = self.map_to_virtual(self.itag.p)

        else:
            self.itag.tracked = False


    def state_timer_callback(self):

        # Get time in seconds
        time, prev_time = self.get_time()

        self.publish_offboard_control_heartbeat(position = True)

        match self.current_state:

            case st.OFFBOARD_ARM:
                if (self.get_vehicle_status() != None and self.get_vehicle_land_detected() != None and self.get_vehicle_attitude() != None):
                    self.info_once("Callback check passed...")
                    self.callback_test = True

                # Wait for callbacks to have fired at least once...
                if self.callback_test:
                    # Very sequential offboard enable and arm sequence
                    if isinstance(self.get_vehicle_status(), px4_msgs.msg.VehicleStatus):
                        # Enable offboard mode if not yet enabled
                        if not self.offboard_enabled:
                            self.engage_offboard_mode()

                        # If it successfully enables we can move on with life
                        if (self.get_vehicle_status().nav_state == px4_msgs.msg.VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                            self.info_once("Successfully entered offboard navigation state")
                            self.offboard_enabled = True

                        # If offboard enabled, arm
                        if self.offboard_enabled & (self.get_vehicle_status().arming_state != px4_msgs.msg.VehicleStatus.ARMING_STATE_ARMED):
                            self.info_once("Arming quadrotor")
                            self.arm(log=False)

                        # Transition to search once offboard is established and quadrotor is armed
                        if self.get_vehicle_status().arming_state == px4_msgs.msg.VehicleStatus.ARMING_STATE_ARMED:
                            self.info_once("PX4 in offboard mode and quadrotor armed")
                            self.transition(ev.ARMED)

                    else:
                        self.info_once("Waiting for get_vehicle_status() to populate...")

            case st.SEARCH:
                self.info_once("Transitioned to SEARCH state")
                # There will probably be more detail here... but for now this will do
                
                self.publish_position_setpoint(0, 0, -4.5)
                
                # Log confidence output
                self.info(str(self.otag.confidence)) if time != prev_time else None
                if self.otag.is_confident(.8):
                    self.info("Outer tag confidence > 0.8: SEARCH -> APPROACH")
                    self.transition(ev.OUT_TARGET_LOCKED)

            case st.APPROACH:
                self.info_once("Transitioned to APPROACH state")

                # Time to start servoing off IBVS
                vel_cmd, err, _ = self.solve_ibvs(self.otag.pv, 100, self.z, self.ogain) # Improve(d) Z estimate! # Use to be 4

                # Update the IBVS command with FF estimate from the lidar
                if (self.otag.filtered_err_within_tol(120)):
                    vel_cmd[2] = vel_cmd[2] - self.z_vel

                self.publish_velocity_setpoint(-vel_cmd[0], -vel_cmd[1], vel_cmd[2], vel_cmd[3])

                # If tags were seen in the last message, update tag. If not, start to worsen filtered err norm.
                for tag in [self.otag, self.itag]:
                   if tag.tracked:
                       tag.update_err_filtered(err)
                   else:
                       tag.update_err_filtered(np.array([[300, 300], [300, 300]]))

                # If not confidently locked on to the outer tag, abort approach and move to SEARCH state
                if not self.otag.is_confident(.6):
                    self.info("Lost tracking on outer tag: APPROACH -> SEARCH")
                    self.transition(ev.OUT_TARGET_LOST)

                self.info(f"Otag err: {self.otag.err_filtered:.1f}, IBVS cmd: {vel_cmd}, Otag confidence: {self.otag.confidence:.1f}") if time != prev_time else None

                # If the error norm has been low for a sufficient amount of time (by simple Bayes filter) move to fine approach
                # Also requires confidence in inner_tag to be above a certain tolerance
                print(self.otag.filtered_err_within_tol(self.otag_err)) if time != prev_time else None
                
                if self.otag.filtered_err_within_tol(self.otag_err):
                   if self.itag.is_confident(.8):
                       self.info("Outer tag stable and inner tag tracked - moving to FINE_APPROACH")
                       self.transition(ev.IN_TARGET_LOCKED)

            case st.FINE_APPROACH:
                self.info_once("Transitioned to FINE_APPROACH state")
                
                # Change IBVS target to inner tag, use more reasonable Z estimate (this is still stupid)
                vel_cmd, err, _ = self.solve_ibvs(self.itag.pv, 60, self.z, self.igain) # Improve Z estimate!
                
                # Update the IBVS command with FF estimate from the lidar
                vel_cmd[2] = vel_cmd[2] - self.z_vel

                self.publish_velocity_setpoint(-vel_cmd[0], -vel_cmd[1], vel_cmd[2], vel_cmd[3])

                # If tags were seen in the last message, update tag. If not, start to worsen filtered err norm.
                for tag in [self.otag, self.itag]:
                    if tag.tracked:
                        tag.update_err_filtered(err)
                    else:
                        tag.update_err_filtered(np.array([[300, 300], [300, 300]]))

                # If not confidently locked on to the outer tag, abort approach and move to SEARCH state
                if not self.itag.is_confident(.2):
                    self.info("Lost tracking on inner tag: FINE_APPROACH -> SEARCH")
                    self.transition(ev.IN_TARGET_LOST)

                self.info(f"Itag err: {self.itag.err_filtered:.1f}, IBVS cmd: {str(vel_cmd)}, Itag confidence: {self.itag.confidence:.1f}") if time != prev_time else None

                # If the error norm has stabilized again, can start landing approach
                if self.itag.err_filtered < self.itag_err:
                    if self.itag.is_confident(.8):
                        self.info_once("Sending it!")
                        self.transition(ev.SENDING_IT)

            case st.LANDING:
                self.info_once("Starting to land!")
                vel_cmd, err, _ = self.solve_ibvs(self.itag.pv, 600, self.z, self.lgain) # Improve Z estimate!

                landing_vz = 2.5 * vel_cmd[2]

                alpha = 0.01
                vel_cmd[2] = (1 - alpha) * self.landing_pvz + alpha * landing_vz
                self.landing_pvz = vel_cmd[2]

                self.info("IBVS vel: " + str(vel_cmd[2])) if time != prev_time else None
                self.publish_velocity_setpoint(-vel_cmd[0], -vel_cmd[1], vel_cmd[2], vel_cmd[3])

                if (self.get_vehicle_land_detected().maybe_landed == True or self.get_vehicle_land_detected().landed == True):
                    self.info_once("Land detected!")
                    self.transition(ev.LAND_DETC)

            case st.LANDED:
                pass

def main(args=None) -> None:
    rclpy.init(args=args)
    
    x500_rpi_test_node = X500()

    try:
        rclpy.spin(x500_rpi_test_node)
    except SystemExit:
        x500_rpi_test_node.destroy_node()

if __name__ == '__main__':
    main()