from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from scipy.spatial.transform import Rotation as R
from custom_msgs.msg import TargetOdometry
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import Bool
from functools import partial
from rclpy.node import Node
import numpy as np
import rclpy

class TargetEKF(Node):

    def __init__(self):

        super().__init__("target_ekf_node")

        self.barge_tag_z_offset = -1.63 # This one is okay to hardcode I think 

        # use mocap as source
        self.declare_parameter("use_mocap_visual_odometry", False)
        self.use_mocap_odom = self.get_parameter("use_mocap_visual_odometry").get_parameter_value().bool_value

        # camera parameters
        self.declare_parameter("cam_x", 0.0)
        self.declare_parameter("cam_y", 0.0)
        self.declare_parameter("cam_z", 0.20)

        self.declare_parameter("cam_roll", 0.0)
        self.declare_parameter("cam_pitch", 0.0)
        self.declare_parameter("cam_yaw", 0.0)

        # Static transformations of the system
        # Camera mounting offsets (degrees)
        cam_roll  = self.get_parameter("cam_roll").get_parameter_value().double_value
        cam_pitch = self.get_parameter("cam_pitch").get_parameter_value().double_value
        cam_yaw   = self.get_parameter("cam_yaw").get_parameter_value().double_value
        self.get_logger().info(f"Camera mounting offsets: roll: {cam_roll}, pitch: {cam_pitch}, yaw: {cam_yaw}", once=True)

        # Camera mounting offsets (meters)
        cam_x = self.get_parameter("cam_x").get_parameter_value().double_value
        cam_y = self.get_parameter("cam_y").get_parameter_value().double_value
        cam_z = self.get_parameter("cam_z").get_parameter_value().double_value
        self.get_logger().info(f"Camera mounting offsets: x: {cam_x}, y: {cam_y}, z: {cam_z}", once=True)

        R_c_m = R.from_euler(
            "XYZ",
            [cam_roll, cam_pitch, cam_yaw],
            degrees=True
        ).as_matrix()

        self.T_c_m = np.eye(4)
        self.T_c_m[:3, :3] = R_c_m

        # Transform from mount to body frame
        self.T_m_b = np.array([[1, 0, 0, cam_x],
                               [0, 1, 0, cam_y],
                               [0, 0, 1, cam_z],
                               [0, 0, 0, 1]], dtype=np.float64)

        ## Dynamic transformations
        # Transform from body to vehicle-1 frame
        self.T_b_v = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]], dtype=np.float64)

        # Transform from vehicle-1 to inertial frame
        self.T_v_i = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]], dtype=np.float64)

        self.R_b_v = np.eye(3, dtype=np.float64)
        self.R_v_b = np.eye(3, dtype=np.float64)

        # Transform from intertial to GPS frame
        self.T_i_gps =  np.array([[1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]], dtype=np.float64)

        # Camera parameters
        self.r_aruco = 0.05
        
        # GPS uncertainty
        self.r_gps = 6.0

        # Initialize euler angles
        self.phi = 0.0 # roll
        self.theta = 0.0 # pitch
        self.psi = 0.0 # yaw

        # Initialize position
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # 4x1 vectors to hold measurement
        self.Z_c = np.array([0,0,0,1], dtype=np.float64).reshape(4,1)
        self.Z_i = np.array([0,0,0,1], dtype=np.float64).reshape(4,1)
        self.Z_gps = np.array([0,0,0,1], dtype=np.float64).reshape(4,1)

        ## EKF Data 

        # State vector
        self.x_hat = np.zeros((4,1), dtype=np.float64)

        # Propagation Jacobian (system model?)
        self.F = np.eye(4, dtype=np.float64)

        # Covariance Matrix
        self.P = np.diag([1.e2, 1.e2, 1.e2, 1.e2])

        # Model input uncertainty
        self.Q = np.diag([1.e0, 1.e0])

        # Gamma(T)
        self.Gamma = np.zeros((4,2), dtype=np.float64)
        
        # Kalman gain.
        self.K = np.zeros((4,2), dtype=np.float64)

        # Measurement Jacobian
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], dtype=np.float64)
        
        # GPS Measurement Jacobian
        self.H_gps = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0]], dtype=np.float64)
    
        # Measurement Uncertainty
        self.R = np.diag([self.r_aruco, self.r_aruco])
        self.R_gps = np.diag([self.r_gps, self.r_gps])

        # Identity
        self.I = np.eye(4, dtype=np.float64)

        # Subscribe to outer tag pose information
        self.qos = QoSProfile(            
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.VOLATILE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1
        )

        # AruCo pose estimate subscriber
        self.otag_pose_subscriber = self.create_subscription(
            PoseStamped, "/tag/otag/camera_aruco_pose_estimate", partial(self.target_callback, tag="otag"), self.qos
        )

        self.itag_pose_subscriber = self.create_subscription(
            PoseStamped, "/tag/itag/camera_aruco_pose_estimate", partial(self.target_callback, tag="itag"), self.qos
        )

        # Target GPS signal
        self.gps_subscriber = self.create_subscription(
           PoseStamped, "/x500/px4_local/barge/gps", self.gps_callback, self.qos
        )

        # Vehicle odometry subscriber - can be px4 estimate OR mocap visual odometry
        if self.use_mocap_odom:
            self.get_logger().info("Using mocap visual odometry as source", once=True)
            self.vehicle_odometry_subscriber = self.create_subscription(
                VehicleOdometry, "/fmu/in/vehicle_visual_odometry", self.vehicle_odometry_callback, self.qos
            )
        else:
            self.get_logger().info("Using PX4 vehicle odometry as source", once=True)
            self.vehicle_odometry_subscriber = self.create_subscription(
                VehicleOdometry, "/fmu/out/vehicle_odometry", self.vehicle_odometry_callback, self.qos
            )
        self.vehicle_odom = None # Holding variable
        self.has_vehicle_odom = False

        # EKF estimate publisher
        self.aruco_pose_mapped_publisher = self.create_publisher(
            TargetOdometry, "/x500/ekf/target_odometry", self.qos
        )

        # Transformed pose publisher (for debugging)
        self.otag_transformed_pose_publisher = self.create_publisher(
            PoseStamped, "/x500/ekf/otag_transformed_pose", self.qos
        )

        self.itag_transformed_pose_publisher = self.create_publisher(
            PoseStamped, "/x500/ekf/itag_transformed_pose", self.qos
        )

        # Control variables
        self.filter_init = False
        self.ready_to_propogate = False
        self.first_time = True

        # Time tracking variables
        self.t_prev = 0.0
        self.dt = np.float64(0.1)

        # State tracking
        self.fusing_otag = False
        self.last_otag_time = 0.0
        self.fusing_itag = False
        self.last_itag_time = 0.0
        self.tag_timeout = 1.0 # Tag timeout parameter

        # Simple z-filter
        self.zf_T = 0.1
        self.zf_weight_otag = 0.5
        self.zf_weight_itag = 0.2
        self.z_hat = 0.0

        # Self propagate parameter
        self.declare_parameter("filter_propogate", False)
        self.filter_propogate = self.get_parameter("filter_propogate").get_parameter_value().bool_value

        # Update rate when not called by callback
        filter_hz = 5
        self.heartbeat_timer = self.create_timer(1/filter_hz, self.filter_callback)


    def target_callback(self, msg: PoseStamped, tag):

        # Get time
        now = self.get_clock().now().nanoseconds / 1e9

        self.Z_c[0][0] = msg.pose.position.x
        self.Z_c[1][0] = msg.pose.position.y
        self.Z_c[2][0] = msg.pose.position.z

        if (tag == "otag"):
            z_dt = now - self.last_otag_time # z filter
            self.last_otag_time = now
            transform_pub = self.otag_transformed_pose_publisher
            
        elif (tag == "itag"):
            z_dt = now - self.last_itag_time # z filter
            self.last_itag_time = now
            transform_pub = self.itag_transformed_pose_publisher

        if self.has_vehicle_odom:

            # Need to transform c to i (camera to inertial frame)    
            self.map_c_to_i(self.Z_c)        

            # Update simple z-height filter
            self.update_z_filter(self.Z_c, z_dt, tag)

            # Publish transformed pose for debugging
            self.publish_transformed_pose(self.Z_i, transform_pub)

            if not self.filter_init:
                self.get_logger().info(f"Init. filter tag: {tag}", once=True)
                self.init_filter(msg)

            if self.ready_to_propogate:
                # Perform EKF update steps
                
                # Propogate step
                self.propogate_step(now)

                # Run measurement update step
                self.update_step(self.Z_i, self.R)

                # Publish
                self.publish_estimate(log=False)
        else:
            self.get_logger().info("EKF: Waiting for vehicle odometry to populate", throttle_duration_sec = .5)

    def gps_callback(self, msg: PoseStamped):
        
        # Get time
        now = self.get_clock().now().nanoseconds / 1e9

        # Get measurement
        self.Z_gps[0][0] = msg.pose.position.x
        self.Z_gps[1][0] = msg.pose.position.y
        self.Z_gps[2][0] = msg.pose.position.z

        if self.has_vehicle_odom and not (self.fusing_otag or self.fusing_itag):      

            if not self.filter_init:
                self.get_logger().info(f"Init. filter GPS", once=True)
                self.init_filter(msg)

            if self.ready_to_propogate:
                # Perform EKF update steps
                
                # Propogate step
                self.propogate_step(now)

                # Run measurement update step
                self.update_step(self.Z_gps, self.R_gps)

                # Publish
                self.publish_estimate(log=False)
        else:
            if not self.has_vehicle_odom:
                self.get_logger().info("EKF: Waiting for vehicle odometry to populate", throttle_duration_sec = .5)

            else:
                self.get_logger().info("EKF: Ignoring GPS as vision system available", throttle_duration_sec = 10)

    def filter_callback(self):

        if (self.filter_init):

            # Callback runs even when target is not tracked
            now = self.get_clock().now().nanoseconds / 1e9
            
            # Do this manually
            if (now - self.last_otag_time < self.tag_timeout):
                self.fusing_otag = True
            else:
                self.fusing_otag = False

            if (now - self.last_itag_time < self.tag_timeout):
                self.fusing_itag = True
            else:
                self.fusing_itag = False

            if (self.filter_init) and (self.filter_propogate):
                # Only run if filter is already started by measurement
                
                # Propogate
                self.propogate_step(now)

                # No measurement to update filter with

                # Publish data
                self.publish_estimate(log=False)

    def init_filter(self, msg: PoseStamped):

        # Check for NaN values on startup
        x = msg.pose.position.x
        y = msg.pose.position.y

        if not np.isfinite(x) or not np.isfinite(y):
            self.get_logger().warn("EKF rejected NaN init. value", throttle_duration_sec=0.5)
            return
        
        # First time initialization
        self.x_hat[0][0] = x
        self.x_hat[1][0] = y

        self.ready_to_propogate = True # Updated
        self.filter_init = True

    def propogate_step(self, t):

        if not self.first_time:
            self.dt = t - self.t_prev
        else:
            self.first_time = False
            self.t_prev = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(f"dt: {self.dt}", once=True)
        self.get_logger().info(f"x_hat preupdate: {self.x_hat}", once=True)

        # Update elements of F and Gamma
        self.F[0][2] = self.dt
        self.F[1][3] = self.dt

        self.get_logger().info(f"F: {self.F}", once=True)

        self.Gamma[0][0] = (self.dt**2.0)/2.0
        self.Gamma[1][1] = (self.dt**2.0)/2.0
        self.Gamma[2][0] = self.dt
        self.Gamma[3][1] = self.dt

        # Propogate state
        self.x_hat = np.dot(self.F, self.x_hat)

        self.get_logger().info(str(f"x_hat postupdate: {self.x_hat}"), once=True)

        # Propogate covariance
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + np.dot(self.Gamma, np.dot(self.Q, self.Gamma.T))

        self.t_prev = t

    def update_step(self, Z_i, R):

        # Compute Kalman gain
        self.K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(np.dot(self.H, np.dot(self.P, self.H.T)) + R)))

        # Update the estimate.
        self.x_hat = self.x_hat + np.dot(self.K, (Z_i[0:2] - self.x_hat[0:2]))

        # Update covariance
        self.P = np.dot((self.I - np.dot(self.K, self.H)), self.P)

    def publish_estimate(self, log):

        x = self.x_hat[0][0]
        y = self.x_hat[1][0]
        vx = self.x_hat[2][0]
        vy = self.x_hat[3][0]

        # separate source for z estimate
        z = self.z_hat

        # Logging
        if log:
            np.set_printoptions(precision=5, suppress=True)
            self.get_logger().info(f"x: {x:.2f}, y: {y:.2f}, z: {z:.2f}, vx: {vx:.2f}, vy: {vy:.2f}", throttle_duration_sec=1.0)

        tag_ekf_pose = TargetOdometry()
        
        # Header info
        tag_ekf_pose.header.stamp = self.get_clock().now().to_msg()
        tag_ekf_pose.header.frame_id = "px4_local"
        tag_ekf_pose.child_frame_id = "target_px4_local_estimate"

        # Fuse data
        tag_ekf_pose.fusing_otag = self.fusing_otag
        tag_ekf_pose.fusing_itag = self.fusing_itag
        tag_ekf_pose.filter_conf = float(123456)

        # Filtered position estimate
        tag_ekf_pose.pose.pose.position.x = x
        tag_ekf_pose.pose.pose.position.y = y
        tag_ekf_pose.pose.pose.position.z = z

        # Filtered velocity estimate
        tag_ekf_pose.twist.twist.linear.x = vx
        tag_ekf_pose.twist.twist.linear.y = vy

        # Velocity magnitude
        tag_ekf_pose.mag = np.sqrt(vx**2 + vy**2)

        self.aruco_pose_mapped_publisher.publish(tag_ekf_pose)

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        self.vehicle_odom = msg
        self.has_vehicle_odom = True

    def map_c_to_i(self, Z_c):
        # Extract info from vehicle odometry
        # Global position (in px4_local)
        self.x, self.y, self.z = self.vehicle_odom.position[0], self.vehicle_odom.position[1], self.vehicle_odom.position[2] 

        # Current attitude in FRD and heading
        [qw, qx, qy, qz] = self.vehicle_odom.q
        rot = R.from_quat([qw, qx, qy, qz], scalar_first=True)
        psi, theta, phi = rot.as_euler("ZYX") # yaw, pitch, roll

        # Transformation from vehicle-1 to inertial frame
        self.T_v_i[0][3] = self.x
        self.T_v_i[1][3] = self.y
        self.T_v_i[2][3] = self.z
        
        # Pre-calc trig values
        sphi = np.sin(phi)
        cphi = np.cos(phi)
        stheta = np.sin(theta)
        ctheta = np.cos(theta)
        spsi = np.sin(psi)
        cpsi = np.cos(psi)

        # Update rotation from vehicle-1 to body frame
        self.R_v_b[0][0] = ctheta * cpsi
        self.R_v_b[0][1] = ctheta * spsi
        self.R_v_b[0][2] = -stheta
        self.R_v_b[1][0] = sphi * stheta *cpsi - cphi * spsi
        self.R_v_b[1][1] = sphi * stheta * spsi + cphi*cpsi
        self.R_v_b[1][2] = sphi * ctheta
        self.R_v_b[2][0] = cphi * stheta * cpsi + sphi * spsi
        self.R_v_b[2][1] = cphi * stheta * spsi - sphi * cpsi
        self.R_v_b[2][2] = cphi * ctheta

        # Transformation from vehicle-1 to inertial frame
        self.T_v_i[0][3] = self.x
        self.T_v_i[1][3] = self.y
        self.T_v_i[2][3] = self.z
        
        # Rotation from body to vehicle-1 frame (transpose)
        self.R_b_v = self.R_v_b.T
        self.T_b_v[0:3,0:3] = self.R_b_v

        # Compute transformation camera to inertial frame
        self.T_c_i = np.dot(self.T_v_i, np.dot(self.T_b_v, np.dot(self.T_m_b, self.T_c_m)))

        # Transform the measurement expressed in the camera frame to be expressed in the inertial frame.
        self.Z_i = np.dot(self.T_c_i, Z_c)

    def publish_transformed_pose(self, Z_i, transform_pub):

        transformed_pose = PoseStamped()
        transformed_pose.header.stamp = self.get_clock().now().to_msg()
        transformed_pose.header.frame_id = "px4_local"
        transformed_pose.pose.position.x = Z_i[0][0]
        transformed_pose.pose.position.y = Z_i[1][0]
        transformed_pose.pose.position.z = Z_i[2][0]

        transform_pub.publish(transformed_pose)

    def update_z_filter(self, Z_i, dt, tag):
        # simple filter for tag-UAV height estimation from multiple tags
        # no velocity estimation needed so not included in EKF
        match tag:
            case "otag":
                self.z_hat = self.z_hat + self.zf_weight_otag * (1-np.exp(-dt/self.zf_T)) * (Z_i[2][0] - self.z_hat)
            case "itag":
                self.z_hat = self.z_hat + self.zf_weight_itag * (1-np.exp(-dt/self.zf_T)) * (Z_i[2][0] - self.z_hat)

def main(args=None):
    # Define and spin up ROS2 node
    rclpy.init()
    node = TargetEKF()
    rclpy.spin(node)

    # Explicitly kill ROS2 node on exit
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()