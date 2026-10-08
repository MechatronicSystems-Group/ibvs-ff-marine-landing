import rclpy
import px4_msgs.msg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from ibvs.ibvs.supporting.quad import Quad
from enum import Enum
from custom_msgs.msg import ArucoDetcs, VelIBVS
import numpy as np
from time import sleep
#from scipy.signal import butter,filtfilt

"""
Rewritten offboard node to create slightly more scalable code, with a working state machine implementation instead of that crazy nonsense from before

STATES
- SEARCH
- FFOLLOW

"""

class st(Enum):
    SEARCH = 1
    FFOLLOW = 2

class ev(Enum):
    OUT_TARGET_LOCKED = 1
    OUT_TARGET_LOST = 2


class OffboardControl(Quad):
    """
    Offboard control node, inheriting functionality from Quad class for basic UAV control - switching modes, takeoff, etc.
    Quad itself is a ROS2 node, so OffboardControl iteratively inherits all the ROS2 Node functionality
    """

    def __init__(self):
        super().__init__()

        # Save init time
        self.start_time = self.get_clock().now()
        self.elapsed_time = 0

        # Create state machine states
        self.states = {
            st.SEARCH: self.t_search_state,
            st.FFOLLOW: self.t_ffollow_state,
        }
        self.current_state = st.SEARCH

        qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            depth = 1
        )

        # Define publishers and subscribers for state-machine logic
        # Drone control publishers created in Quad class when super().__init__() is called

        # Aruco detections subscriber
        self.aruco_detcs_subscription = self.create_subscription(
            ArucoDetcs, "/aruco_detcs", self.aruco_detcs_callback, qos
        )

        # IBVS command subscriber
        self.ibvs_subscription = self.create_subscription(
            VelIBVS, "/ibvs", self.ibvs_callback, qos
        )

        # Search height
        self.search_height = -2.5

        # IBVS variables
        self.ibvs_vel_outer = None
        self.ibvs_vel_inner = None

        # Variables controlling state machine behavior
        self.begin_out_lock = False                     # Variable to start target lock in aruco_detc callback function
        self.begin_in_lock = False                      # Variable to start target lock in aruco_detc callback function
        self.lock_frames = 500                          # Number of successive frames required for target to be considered locked on
        self.out_lock = False                           # True/False whether outer target is considered locked
        self.out_lock_cnt = 0                           # Array to store succesful ID's for outer tag
        self.in_lock = False                            # True/False whether inner target is consdered locked
        self.in_lock_cnt = 0                            # Array tp store succesful ID's for inner tag
        self.out_tag_fro_tol = 350                     # Frobenius norm error tolerance for starting inner tag lock on
        self.out_tag_fro_tol_frames = 100               # Number of consecutive frames Frob. error must be below tolerance
        self.out_tag_fro_tol_arr = np.ones(self.out_tag_fro_tol_frames) * 10e9

        # Define a counter to handle state-machine logic 
        self.state_timer = self.create_timer(0.02, self.state_timer_callback)

        # Support variable for resettable print_once function
        self.printed_flags = set() 

    # Layout for class definitions
    # Utility functions -> Transition functions -> Callback functions

    def info_once(self, message: str, flag: str):
        if flag not in self.printed_flags:
            self.info(message)
            self.printed_flags.add(flag)

    def reset_flag(self, flag):
        self.printed_flags.discard(flag)

    def get_time(self):
        """
        Function to get the current time, which also returns the previous time for use in conditional info logging
        Outputs time and prev_time in seconds, not nanoseconds
        """
        self.prev_time = self.elapsed_time
        self.elapsed_time = int((self.get_clock().now() - self.start_time).nanoseconds / 1e9)

        return self.elapsed_time, self.prev_time

    def get_err_norm(self, err: VelIBVS.inner_err):
        """
        Calculates Frobenius norm of error array
        """
        err_vec = np.array([[err.p1_err.x, err.p1_err.y],
                            [err.p2_err.x, err.p2_err.y],
                            [err.p3_err.x, err.p3_err.y],
                            [err.p4_err.x, err.p4_err.y]])

        err_norm = np.linalg.norm(err_vec, 'fro')
        return err_norm
        
    def transition(self, event):
        """
        Transitions out of a state based on a passed event, where the reaction to the event is 
        handled by a dedicated 't_' function to handle different events that can plausibly happen while in a given state.
        As defined in self.states each system state has to have an associated event handling function - with given condition(s)
        to transit from itself to different state by setting self.current_state
        """
        if event in [ev.OUT_TARGET_LOCKED, ev.OUT_TARGET_LOST]:
            # Index transition functions with current state, then run the function with arg = event that fired
            self.states[self.current_state](event)

    def t_search_state(self, event):
        if event == ev.OUT_TARGET_LOCKED:
            self.info("Outer target lock acquired: SEARCH -> FFOLLOW")
            self.current_state = st.FFOLLOW

        else:
            self.info(f"t_search_state given unexpected event: {event}")

    def t_ffollow_state(self, event):
        if event == ev.OUT_TARGET_LOST:
            self.info("Outer target lock lost: FFOLLOW -> SEARCH")
            self.current_state = st.SEARCH

    def aruco_detcs_callback(self, msg: ArucoDetcs):
        """
        Callback function used to detect if a tag can be considered "locked on" by being successively ID'd for a set number of frames, defined in __init__ function
        
        self.out_lock_cnt and self.in_lock_cnt acts as "confidence" for ArUco tag detection - adds up for every frame identified, but exponentially decreases for every
        frame it is _not_ detected.
        """

        self.out_tag_detc = msg.outer_tag_detc
        self.in_tag_detc = msg.inner_tag_detc

        if (self.begin_out_lock):
            # Handle outer tag lock
            if msg.outer_tag_detc == True:
                self.out_lock_cnt += 1
            else:
                self.out_lock_cnt = int(self.out_lock_cnt * 0.5) # halve 'confidence' if outer tag is not detected for frame
                # self.info(f"Halved out_lock_cnt: {self.out_lock_cnt}")

        if (self.begin_in_lock):
            # Handle inner tag lock
            if msg.inner_tag_detc == True:
                self.in_lock_cnt += 1
            else:
                self.in_lock_cnt = int(self.in_lock_cnt * 0.5) # halve 'confidence' if inner tag is not detected for frame
               # self.info(f"Halved in_lock_cnt: {self.in_lock_cnt}")

        # If either count reaches 100 assume target is found
        if (self.out_lock_cnt > 100):
            self.info("Gained target lock on outer tag") if self.out_lock ==  False else None
            self.out_lock = True # This is and below is separate to event - used for triggering events but is not one itself
            self.transition(ev.OUT_TARGET_LOCKED)

        elif(self.out_lock == True) and (self.out_lock_cnt < 100):
            # self.info("Outer target lost after previously being found! Triggering event.")
            self.transition(ev.OUT_TARGET_LOST)

    def ibvs_callback(self, msg: VelIBVS):
        """
        Callback function to update given velocity command from IBVS node, which co-subscribes to ArucoDetcs
        """
        self.ibvs_vel_inner = msg.inner_velocity
        self.ibvs_vel_outer = msg.outer_velocity
        self.ibvs_err_inner = msg.inner_err
        self.ibvs_err_outer = msg.outer_err

    def state_timer_callback(self):

        # Get time
        time, prev_time = self.get_time()

        # Publish offboard heartbeat
        self.publish_offboard_control_heartbeat(velocity = True)
        self.publish_position_setpoint(0, 0, z = self.search_height)
        
        match self.current_state:

            case st.SEARCH:

                # Arm and enable offboard if not already
                if (isinstance(self.get_vehicle_status(), px4_msgs.msg.VehicleStatus)):
                    while (self.get_vehicle_status().nav_state != px4_msgs.msg.VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                        
                        self.publish_offboard_control_heartbeat(velocity = True)
                        self.engage_offboard_mode()
                        sleep(0.1)

                    if (self.get_vehicle_status().arming_state != px4_msgs.msg.VehicleStatus.ARMING_STATE_ARMED):
                        self.arm()
                        sleep(0.1)

                    # Takeoff to search height
                    # Also serves as return height if target is lost during landing manoeuvre
                    self.publish_position_setpoint(0, 0, z = self.search_height)

                    # Behaviour in search mode (wait a while before starting new search)
                    search_startup = 5

                    if time < search_startup:
                        self.info(f"Waiting to begin lock, time: {time}") if time != prev_time else None
                    elif time > search_startup:
                        self.begin_out_lock = True
                        self.info_once("Beginning lock on to outer target", "out_tar_lock_start")
                        self.info(f"Outer lock confidence: {self.out_lock_cnt}") if time != prev_time else None

                else:
                    self.info("Waiting for vehicle status information") if time != prev_time else None

            case st.FFOLLOW:

                self.info_once(f"Transitioned to FFOLLOW state", "bro_app_trans")
                if (self.out_lock) and (self.ibvs_vel_outer is not None):
                    # self.info("Publishing velocity command wrt outer target") if time != prev_time else None

                    # If we assume that the IBVS can keep the drone aligned with the target, we can just feed in an offset for the feed forward velocity
                    # Platform velocity is currently 0.10 m/s 
                    # ff_vel = 0.10
                    ff_vel = 0

                    self.publish_velocity_setpoint(-self.ibvs_vel_outer.vx, (-self.ibvs_vel_outer.vy) + (ff_vel), self.ibvs_vel_outer.vz, self.ibvs_vel_outer.vyaw)
                    #self.publish_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
                    # Logging velocity commands to see where the bumps are coming from
                    #self.info(f"vx: {-self.ibvs_vel_outer.vx}, vy: {(-self.ibvs_vel_outer.vy) + (ff_vel)}, vz: {self.ibvs_vel_outer.vz}, vyaw: {self.ibvs_vel_outer.vyaw}")

            case _:
                # Should never get here, represents self.current_state not in st Enum
                self.error("Match/case given unrecognized state variable")

def main(args=None) -> None:
    print('Starting offboard mode node')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)