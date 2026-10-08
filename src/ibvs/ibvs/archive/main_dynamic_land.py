from ibvs.quad import ArucoQuad, Tag
from estimate import VelocityEstimator
from custom_msgs.msg import ArucoDetcs
from sensor_msgs.msg import NavSatFix
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum
import pymap3d as pm
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

class X500(ArucoQuad):

    def __init__(self):
        # Init. functionality of various subclasses
        super().__init__()

        # Create state machine states
        self.states = {
            st.SEARCH: self.t_search_state,
            st.APPROACH: self.t_approach_state,
            st.APPROACH_CLOSE: self.t_close_approach_state,
            st.APPROACH_FF: self.t_ff_approach_state,
            st.LANDED: self.t_landed_state,
            st.OFFBOARD_ARM: self.t_offboard_arm_state,
            st.LANDING: self.t_landing_state
        }
        self.current_state = st.OFFBOARD_ARM

        # Search height to begin with
        self.search_height = -15

        # Start up variables
        self.offboard_enabled = False
        self.callback_test = False
        self.home_gps_coords = None

        # Time tracking variable
        self.start_time = self.get_clock().now()
        self.elapsed_time = 0
        self.last_time = None

        # IBVS objects, functions and booleans
        self.otag = Tag()
        self.itag = Tag()

        self.ogain = [-0.5, -0.5, 0.3, 0.4] # IBVS outer tag gain
        self.igain = [-0.5, -0.5, 0.12, 0.2] # IBVS inner tag gain
        self.lgain = [-0.6, -0.4, 1.0, 0.2] # IBVS inner tag gain
        self.set_camera_parameters() # Set camera parameters

        self.otag_err = 120
        self.itag_err = 80

        self.otag_deque_length = 80
        self.otag_err_moving_avg = deque(maxlen=self.otag_deque_length)

        self.itag_deque_length = 50
        self.itag_err_moving_avg = deque(maxlen=self.itag_deque_length)

        self.search_otag_confidence_threshold = 0.95
        self.approach_otag_confidence_threshold = 0.95

        self.otag_err_moving_avg_threshold = 1.00

        # Subscribe to ArucoDetcs
        self.aruco_detcs_subscriber = self.create_subscription(
            ArucoDetcs, "/aruco_detcs", self.aruco_callback, QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT, 
                durability=DurabilityPolicy.VOLATILE, 
                history=HistoryPolicy.KEEP_ALL, 
                depth=1
            )
        )

        # Subscribe to barge GPS position
        self.barge_gps_subscriber = self.create_subscription(
            NavSatFix, "/barge/gps", self.barge_gps_callback, QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.VOLATILE,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
            )
        )
        self.last_gps_positions = deque(maxlen=5)
        self.avg_gps_position = None

        # Velocity estimator & feed-forward velocity variables
        self.vel_estimator = None
        self.precise_prev_time = 0.0

        self.vel_est_deque_length = 200
        self.vel_est_moving_avg = deque(maxlen=self.vel_est_deque_length)

        self.ff_velocity = 0.0
        self.ff_it = 0

        # Landing variables
        self.landing_pvz = 0
        self.landing_start_time = None

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
            self.current_state = st.APPROACH_CLOSE

        else:
            self.transition_error(event)

    def t_close_approach_state(self, event):

        if event == ev.IN_TARGET_LOST:
            self.info("Inner target lock lost: CLOSE_APPROACH -> SEARCH")
            self.current_state = st.SEARCH

        if event == ev.SENDING_IT:
            self.info("Inner target stable, sending it: CLOSE_APPROACH -> LANDING")
            self.current_state = st.LANDING

        if event == ev.FF_STABLE:
            self.info("FF stabilized, initializing FF state: CLOSE_APPROACH -> FF_APPROACH")
            self.current_state = st.APPROACH_FF

        else:
            self.transition_error(event)

    def t_ff_approach_state(self, event):
        if event == ev.IN_TARGET_LOST:
            self.info("Inner target lock lost: CLOSE_APPROACH -> SEARCH")
            self.current_state = st.SEARCH

        if event == ev.IN_TARGET_CENTRE:
            self.info("Centered on inner tag: APPROACH_FF -> LANDING")
            self.current_state = st.LANDING

    def t_landing_state(self, event):

        if event == ev.LAND_DETC:
            self.info("Land detected: LANDING -> LANDED")
            self.current_state = st.LANDED

        else:
            self.transition_error(event)

    def t_landed_state(self, event):
        # This should not receive any events (right now anyway)
        self.transition_error(event)

    def barge_gps_callback(self, msg: NavSatFix):
        # Store last n GPS messages
        self.last_gps_positions.append((msg.latitude, msg.longitude, msg.altitude))

        if len(self.last_gps_positions) == 0:
            return

        arr = np.array(self.last_gps_positions)
        mean = arr.mean(axis=0)

        self.avg_gps_position = mean

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

        t_now = self.get_clock().now().nanoseconds / 10e8
        dt = t_now - self.precise_prev_time
        self.precise_prev_time = t_now

        self.publish_offboard_control_heartbeat(position = True)

        match self.current_state:


            case st.OFFBOARD_ARM:
                # This is getting stupid, rewrite this
                if (self.get_vehicle_status() != None and self.get_vehicle_land_detected() != None and self.get_vehicle_attitude() != None and self.get_vehicle_global_position() != None):
                    self.info_once("Callback check passed...")
                    self.callback_test = True

                if self.callback_test and self.home_gps_coords == None:
                    # Save home position
                    self.info("Setting home GPS position")
                    #self.home_gps_coords = [self.get_vehicle_global_position().lat, self.get_vehicle_global_position().lon, self.get_vehicle_global_position().alt]
                    # Disgusting hardcode while bug persists
                    self.home_gps_coords = [36.0235858, 129.37814230, 2.00]

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
                # There will probably be more detail here... but for now this will do
                
                if type(self.avg_gps_position) != type(None):
                    target_lat = self.avg_gps_position[0]
                    target_lon = self.avg_gps_position[1]

                    # Get drone GPS coordinates
                    vehicle_gpos = self.get_vehicle_global_position()

                    v_lat = vehicle_gpos.lat
                    v_lon = vehicle_gpos.lon

                    err_lat = self.avg_gps_position[0] - v_lat
                    err_lon = self.avg_gps_position[1] - v_lon

                    # Need to convert the target GPS coordinates to local coordinates
                    n, e, d = pm.geodetic2ned(
                        lat=target_lat,
                        lon=target_lon,
                        h=-self.search_height,
                        lat0=self.home_gps_coords[0],
                        lon0=self.home_gps_coords[1],
                        h0=self.home_gps_coords[2]
                    )

                    # Publish position setpoint
                    self.publish_position_setpoint(x=n, y=e, z=d, yaw=np.pi)

                    if time != prev_time:
                        print(self.diag_block(
                            state=self.current_state,
                            otag_confidence=f"{self.otag.confidence:.2f}",
                            otag_confident_threshold=f"{self.search_otag_confidence_threshold}",
                            boat_lat=f"{target_lat}",
                            boat_lon=f"{target_lon}",
                            vehicle_lat=f"{v_lat}",
                            vehicle_lon=f"{v_lon}",
                            err_lat=f"{err_lat}",
                            err_lon=f"{err_lon}",
                            e_n_u=f"{n:.2f}, {e:.2f}, {d:.2f}"
                        ))
                
                else:
                    self.publish_position_setpoint(25, 0, -15)

                if self.otag.is_confident(self.search_otag_confidence_threshold):
                    self.info(f"Outer tag confidence > {self.search_otag_confidence_threshold}: SEARCH -> APPROACH")
                    self.transition(ev.OUT_TARGET_LOCKED)


            case st.APPROACH:

                # Time to start servoing off IBVS
                vel_cmd, err, _, J = self.solve_ibvs(self.otag.p, 50, 7, self.ogain) # Improve(d) Z estimate! # Use to be 4

                self.publish_velocity_setpoint(vel_cmd[0], vel_cmd[1], vel_cmd[2], vel_cmd[3])

                # If tags were seen in the last message, update tag. If not, start to worsen filtered err norm.
                for tag in [self.otag, self.itag]:
                   if tag.tracked:
                       tag.update_err_filtered(err)
                   else:
                       tag.update_err_filtered(np.array([[300, 300], [300, 300]]))
                self.otag_err_moving_avg.append(self.otag.err_filtered)

                # If not confidently locked on to the outer tag, abort approach and move to SEARCH state
                if not self.otag.is_confident(.30):
                    self.info("Lost tracking on outer tag: APPROACH -> SEARCH")
                    self.transition(ev.OUT_TARGET_LOST)

                # Check for stability, target confidence, otag populated before transition to close approach
                
                pop, stable, conf = False, False, False

                if len(self.otag_err_moving_avg) == self.otag_deque_length:
                    pop = True

                if (np.std(self.otag_err_moving_avg)) < self.otag_err_moving_avg_threshold:
                    stable = True

                if self.otag.is_confident(self.approach_otag_confidence_threshold) and self.itag.is_confident(.9):
                    conf = True

                self.info(f"Pop: {pop}, Error signal stable: {stable}, Tags: {conf}") if time != prev_time else None

                if time != prev_time:
                    print(self.diag_block(
                        state=self.current_state,
                        ibvs_command=f"x: {vel_cmd[0]:.2f}, y: {vel_cmd[1]:.2f}, z: {vel_cmd[2]:.2f}, yaw: {vel_cmd[3]:.2f}",
                        otag_deque_length=f"{len(self.otag_err_moving_avg)}",
                        otag_deque_length_threshold=f"{self.otag_deque_length}",
                        otag_err_moving_avg=f"{np.std(self.otag_err_moving_avg)}",
                        otag_err_moving_avg_threshold=f"{self.otag_err_moving_avg_threshold}",
                        otag_confidence=f"{self.otag.confidence:.2f}",
                        otag_confident_threshold=f"{self.approach_otag_confidence_threshold}",
                        pop_stable_conf=f"{pop}, {stable}, {conf}",
                        itag_confidence=f'{self.itag.confidence:.1f}'
                    ))

                if pop and stable and conf:
                    self.info("Checks passed, moving to inner target tracking")
                    self.transition(ev.IN_TARGET_LOCKED)


            case st.APPROACH_CLOSE:
                self.info_once("Transitioned to FINE_APPROACH state")
                
                # Change IBVS target to inner tag, use more reasonable Z estimate (this is still stupid)
                vel_cmd, err, _, J = self.solve_ibvs(self.itag.p, 40, 4, self.igain) # Improve Z estimate!

                self.publish_velocity_setpoint(vel_cmd[0], vel_cmd[1], vel_cmd[2], vel_cmd[3])

                # If tags were seen in the last message, update tag. If not, start to worsen filtered err norm.
                for tag in [self.otag, self.itag]:
                    if tag.tracked:
                        tag.update_err_filtered(err)
                    else:
                        tag.update_err_filtered(np.array([[300, 300], [300, 300]]))
                self.itag_err_moving_avg.append(self.itag.err_filtered)

                # If not confidently locked on to the outer tag, abort approach and move to SEARCH state
                if not self.itag.is_confident(.2):
                    self.info("Lost tracking on inner tag: FINE_APPROACH -> SEARCH")
                    self.transition(ev.IN_TARGET_LOST)

                if len(self.itag_err_moving_avg) == self.itag_deque_length:
                    self.info_once("Itag err array populated, beginning to track err std. dev.")
                    if (np.std(self.itag_err_moving_avg) < 0.40) and self.vel_estimator is None:
                        self.info("Initializing velocity estimator")
                        self.vel_estimator = VelocityEstimator()

                if self.vel_estimator is not None:
                    # Get current quad velocity
                    local_pos = self.get_vehicle_local_position()
                    Vc = np.array([local_pos.vx, local_pos.vy, local_pos.vz, 0.0])
                    self.vel_estimator.update_filter(dt=dt, err=err, V_cam=Vc, J=J)

                    # Print out current estimate
                    current_vel_est = self.vel_estimator.Vp_hat
                    self.vel_est_moving_avg.append(current_vel_est)
                    #self.info(str(current_vel_est)) if time != prev_time else None

                    # Currently working on limited implementation so only look at x
                    if len(self.vel_est_moving_avg) >= self.vel_est_deque_length:
                        x = np.std([arr[0] for arr in self.vel_est_moving_avg])
                        
                        if x < 0.05:
                            self.info("FF velocity estimate stable, enabling FF in FF_APPROACH state")
                            self.ff_it = 0 # Reset in case of multiple attempts
                            self.transition(ev.FF_STABLE)


            case st.APPROACH_FF:
                self.info_once("Transitioned to APPROACH_FF state")
                
                # Change IBVS target to inner tag, use more reasonable Z estimate (this is still stupid)
                vel_cmd, err, _, J = self.solve_ibvs(self.itag.p, 60, 4, self.igain) # Improve Z estimate!

                # If tags were seen in the last message, update tag. If not, start to worsen filtered err norm.
                for tag in [self.otag, self.itag]:
                    if tag.tracked:
                        tag.update_err_filtered(err)
                    else:
                        tag.update_err_filtered(np.array([[300, 300], [300, 300]]))
                self.itag_err_moving_avg.append(self.itag.err_filtered)

                # If not confidently locked on to the outer tag, abort approach and move to SEARCH state
                if not self.itag.is_confident(.2):
                    self.info("Lost tracking on inner tag: FINE_APPROACH -> SEARCH")
                    self.transition(ev.IN_TARGET_LOST)

                # Get current velocity estimate
                current_vel_est = self.vel_estimator.Vp_hat

                # Handle FF velocity. Need to blend in FF _slowly_ so that the filter does not get excited by sudden kick
                tau = 10
                self.ff_it += 1
                scale_factor = min(100.0, self.ff_it * 100 / (tau * self.state_machine_hz)) / 100

                # Blend in FF velocity SLOWLY to avoid destabilizing estimator
                vel_cmd_ff = vel_cmd[0] + scale_factor * current_vel_est[0]

                self.publish_velocity_setpoint(vel_cmd_ff, vel_cmd[1], vel_cmd[2], vel_cmd[3])

                # Diagnostics readout
                if time != prev_time:
                    print(self.diag_block(
                        state=self.current_state,
                        ff_scale=f"{scale_factor:.2f}",
                        x_ibvs=f"{vel_cmd[0]:.2f}",
                        x_est=f"{current_vel_est[0]:.2f}",
                        tau=tau,
                        itag_err_filtered=f"{self.itag.err_filtered:.1f}",
                        itag_is_confident=f"{self.itag.is_confident(0.40)}"
                    ))

                # If we are close we can land!
                if self.itag.err_filtered < 35:
                    self.info_once("Itag centered")
                    if self.itag.is_confident(0.40):
                        self.info_once("Itag close + on centre begin landing")
                        self.transition(ev.IN_TARGET_CENTRE)


            case st.LANDING:
                self.info_once("Starting to land!")

                if type(self.landing_start_time) == type(None):
                    self.landing_start_time = self.get_clock().now().nanoseconds / 10e8

                landing_time = 5.0

                starting_offset = 70
                ending_offset = 500
                elapsed_time = self.get_clock().now().nanoseconds / 10e8 - self.landing_start_time

                # Figure out scaling factor
                scale_factor = min(1.0, elapsed_time / landing_time)

                offset = starting_offset + scale_factor * (ending_offset - starting_offset)

                # Change IBVS target to inner tag, use more reasonable Z estimate (this is still stupid)
                vel_cmd, err, _, J = self.solve_ibvs(self.itag.pv, offset, 4, self.igain) # Improve Z estimate!

                # Get current velocity estimate
                current_vel_est = self.vel_estimator.Vp_hat
                vel_cmd_ff = vel_cmd[0] + current_vel_est[0]

                # Scale landing velocity
                # landing_vz = 3.5 * vel_cmd[2]

                # alpha = 0.02
                # vel_cmd[2] = (1 - alpha) * self.landing_pvz + alpha * landing_vz
                # self.landing_pvz = vel_cmd[2]

                landing_velocity = 0.8
                vel_cmd[2] = landing_velocity # Overwrite landing velocity with just go down

                # self.info("IBVS vel: " + str(vel_cmd[2])) if time != prev_time else None
                self.publish_velocity_setpoint(vel_cmd_ff, vel_cmd[1], vel_cmd[2], vel_cmd[3])

                if time != prev_time:
                    print(self.diag_block(
                        state=self.current_state,
                        offset=f"{offset:.1f}",
                        landing_z_vel=f"{vel_cmd[2]:.1f}",
                        maybe_landed = f"{self.get_vehicle_land_detected().maybe_landed}",
                        landed = f"{self.get_vehicle_land_detected().landed}"
                    ))

                if (self.get_vehicle_land_detected().maybe_landed == True or self.get_vehicle_land_detected().landed == True):
                    self.info_once("Land detected!")
                    self.transition(ev.LAND_DETC)


            case st.LANDED:
                pass

    def diag_block(self, state, **entries):
        """
        Simple constructor for better readouts going forward.

        Use example:
        print(self.diag_block(
            state=self.current_state,
            ff_scale=f"{scale_factor:.2f}",
            x_ibvs=f"{vel_cmd[0]:.2f}",
            x_est=f"{current_vel_est[0]:.2f}",
            tau=tau
            )
        )

        TODO: Possibly consider adding logging/ROS2 integration
        """

        lines = ["\n", f"DIAGNOSTICS: {state}", "---------------"]
        for k, v in entries.items():
            lines.append(f"{k}: {v}")
        body = "\n".join(lines)

        # Return text wrapped with newlines
        return f"{body}"

def main(args=None) -> None:
    rclpy.init(args=args)
    
    x500_rpi_test_node = X500()

    try:
        rclpy.spin(x500_rpi_test_node)
    except SystemExit:
        x500_rpi_test_node.destroy_node()

if __name__ == '__main__':
    main()