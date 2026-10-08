from quad import ArucoQuad, Tag
from custom_msgs.msg import ArucoDetcs
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
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
    """
    Offboard control node, inheriting functionality from Quad class for basic UAV control - switching modes, takeoff, etc.
    Inherits specific IBVS and aruco tag functionality from superclass ArucoQuad of class Quad
    Quad itself is a ROS2 node, so OffboardControl iteratively inherits all the ROS2 Node functionality
    """
    def __init__(self):
        # Init. functionality of various subclasses
        super().__init__()

        # Big important parameters
        self.gain = 0.1
        self.out_tag_fro_tol = 65
        self.in_tag_fro_tol = 90

        # Save init time
        self.start_time = self.get_clock().now()
        self.elapsed_time = 0

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


        # Initial landing offset
        self.landing_offset = 150
        self.prev_landing_z_vel = 0

        # Set camera parameters (with defaults)
        self.set_camera_parameters()

        # Start a timer for state behaviour - 100 Hz
        self.state_timer = self.create_timer(0.01, self.state_timer_callback)

        # Various boolean and other variables
        self.offboard_enabled = False
        self.aruco_callback_enabled = False
        self.feedforward_zfilter = False

        # Initiate tag objects
        self.outer_tag = Tag()
        self.inner_tag = Tag()

    """
    Layout for class functiosn:
    Utility functions -> Transition functions -> Callback functions
    """

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

    def aruco_callback(self, msg: ArucoDetcs):

        # Test mapping code
        # Returns array of Point classes with x,y values
        out_tag_virtual, in_tag_virtual = self.map_to_virtual(msg)

        # Only run if tag detected:
        if msg.outer_tag_detc:
            self.outer_tag.tracked = True
            # Update tag confidence
            self.outer_tag.update_confidence(msg.outer_tag_detc)

            # Update outer aruco tag * raw points *
            self.outer_tag.p1, self.outer_tag.p2, self.outer_tag.p3, self.outer_tag.p4 = msg.outer_tag.p1, msg.outer_tag.p2, msg.outer_tag.p3, msg.outer_tag.p4

            # Update outer aruco tag * mapped points *
            for i, value in enumerate(out_tag_virtual, start=1):
                setattr(self.outer_tag, f"p{i}v", value)

        else:
            # let state machine know tag was not seen
            self.outer_tag.tracked = False

        if msg.inner_tag_detc:
            self.inner_tag.tracked = True
            # Update tag confidence
            self.inner_tag.update_confidence(msg.inner_tag_detc)

            # Update inner aruco tag * raw points *
            self.inner_tag.p1, self.inner_tag.p2, self.inner_tag.p3, self.inner_tag.p4 = msg.inner_tag.p1, msg.inner_tag.p2, msg.inner_tag.p3, msg.inner_tag.p4
        
            # Update inner aruco tag * mapped points *
            for i, value in enumerate(in_tag_virtual, start=1):
                setattr(self.inner_tag, f"p{i}v", value)

        else:
            # let state machine know tag was not seen
            self.inner_tag.tracked = False

    def state_timer_callback(self):

        # Get time in seconds
        time, prev_time = self.get_time()
        
        # Publish offboard control heartbeat
        self.publish_offboard_control_heartbeat(velocity = True)

        match self.current_state:


            case st.OFFBOARD_ARM:
            
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
                
                self.publish_position_setpoint(1, 1, -5)

                #if not self.aruco_callback_enabled:
                #    self.start_aruco_callback()
                
                # Log confidence output
                self.info(str(self.outer_tag.confidence)) if time != prev_time else None
                if self.outer_tag.is_confident(.8):
                    self.info("Outer tag confidence > 0.8: SEARCH -> APPROACH")
                    self.transition(ev.OUT_TARGET_LOCKED)


            case st.APPROACH:
                self.info_once("Transitioned to APPROACH state")

                self.publish_position_setpoint(0, 0, -4)
                self.start_lidar_velocity_estimate_callback()

                if isinstance(self.get_zfilter(), Odometry):
                    print(self.get_zfilter().twist.twist.linear.z)

                # Time to start servoing off IBVS
                vel_cmd, err, goal_points = self.solve_ibvs(self.outer_tag.get_virtual_points(), 140, 2.5) # Improve Z estimate!
                z_vel = vel_cmd[2]

                # Wait for decent confidence on outer tag to start zfilter callback
                if self.outer_tag.filtered_err_within_tol(150):
                    self.info_once("Starting zfilter callback + velocity matching")
                    if self.zfilter_odom == None:
                        self.start_lidar_velocity_estimate_callback()
                        self.feedforward_zfilter = True

                if self.feedforward_zfilter == True:
                    if isinstance(self.get_zfilter(), Odometry):
                        self.info_once("Beginning to IBVS with Z filter feedforward now")
                        # Add feedforward term from zfilter to velocity command to non-reactively match platform z-height
                        vel_cmd[2] = vel_cmd[2] - self.get_zfilter().twist.twist.linear.z # No feedforward?

                        self.info(f"z_filter: {self.get_zfilter().twist.twist.linear.z}, z_ibvs: {z_vel}")
                    
                    else:
                        self.info("Waiting for zfilter to populate") if time != prev_time else None

                self.publish_velocity_setpoint(-vel_cmd[0], -vel_cmd[1], vel_cmd[2], vel_cmd[3])

                # Publish what the different commands are
                
                self.info(str(vel_cmd)) if time != prev_time else None
                self.info(f"Filtered outer tag frobenius err norm: {str(self.outer_tag.err_filtered)}") if time != prev_time else None

                # If tags were seen in the last message, update tag. If not, start to worsen filtered err norm.
                for tag in [self.outer_tag]:
                   if tag.tracked:
                       tag.update_err_filtered(err)
                   else:
                       tag.update_err_filtered(np.array([[300, 300], [300, 300]]))
                
                # If the error norm has been low for a sufficient amount of time (by simple Bayes filter) move to fine approach
                # Also requires confidence in inner_tag to be above a certain tolerance
                if self.outer_tag.filtered_err_within_tol(self.out_tag_fro_tol):
                   if self.inner_tag.is_confident(.8):
                       self.info("Outer tag stable and inner tag tracked - moving to FINE_APPROACH")
                       self.transition(ev.IN_TARGET_LOCKED)


            case st.FINE_APPROACH:

                self.info_once("Transitioned to FINE_APPROACH state")
                
                # Change IBVS target to inner tag, use more reasonable Z estimate (this is still stupid)
                vel_cmd, err, goal_points = self.solve_ibvs(self.inner_tag.get_virtual_points(), 115, 0.75) # Improve Z estimate!
                
                self.publish_velocity_setpoint(-vel_cmd[0], -vel_cmd[1], vel_cmd[2], vel_cmd[3])

                self.info(str(vel_cmd)) if time != prev_time else None
                self.info(f"Filtered inner tag frobenius err norm: {str(self.inner_tag.err_filtered)}") if time != prev_time else None
                
                for tag in [self.outer_tag, self.inner_tag]:
                    if tag.tracked:
                        tag.update_err_filtered(err)
                    else:
                        tag.update_err_filtered(np.array([[300, 300], [300, 300]]))

                self.info(str(self.inner_tag.err_filtered < self.in_tag_fro_tol)) if time != prev_time else None
                if self.inner_tag.err_filtered < self.in_tag_fro_tol:
                    self.info("good") if time != prev_time else None
                    if self.inner_tag.is_confident():
                        self.info_once("Sending it!")
                        self.transition(ev.SENDING_IT)

            case st.LANDING:
                self.info_once("Starting to land!")
                vel_cmd, err, goal_points = self.solve_ibvs(self.inner_tag.get_virtual_points(), 600, 1) # Improve Z estimate!

                self.info("IBVS vel: " + str(vel_cmd[2])) if time != prev_time else None

                # Basically a crummy Bayes filter on landing to smooth things out so we don't plummet
                damping = 0.005
                l_factor = 8
                cmd_landing_z_vel = l_factor * vel_cmd[2]

                landing_z_vel = ((1 - damping) * self.prev_landing_z_vel + damping * cmd_landing_z_vel)
                self.prev_landing_z_vel = landing_z_vel

                self.info("Filtered vel: " + str(landing_z_vel)) if time != prev_time else None
                self.publish_velocity_setpoint(-vel_cmd[0], -vel_cmd[1], landing_z_vel, vel_cmd[3])

                if (self.get_vehicle_land_detected().maybe_landed == True or self.get_vehicle_land_detected().landed == True):
                    self.info_once("Land detected!")
                    self.transition(ev.LAND_DETC)

            case st.LANDED:
                pass

# def main(args=None) -> None:
#     rclpy.init(args=args)
#     test_node = X500()
#     executor = MultiThreadedExecutor(num_threads = 8)
#     executor.add_node(test_node)
#     executor.spin()

def main(args=None) -> None:
    rclpy.init(args=args)
    
    x500_rpi_test_node = X500()

    try:
        rclpy.spin(x500_rpi_test_node)
    except SystemExit:
        x500_rpi_test_node.destroy_node()

if __name__ == '__main__':
    main()