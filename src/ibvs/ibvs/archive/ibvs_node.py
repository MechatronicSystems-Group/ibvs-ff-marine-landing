#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from custom_msgs.msg import ArucoDetcs, ArucoTag, VelIBVS
import numpy as np
from numpy.linalg import pinv
from time import sleep

"""
This (and the offboard_node) will need SERIOUS modifications to be ready to actually fly in case of dropped messages, undetected corners, outliers, etc.
TODO: Case where drone gets close to platform and loses imaging of corners is currently a serious issue and will cause unknown behaviour
"""

class IBVSNode(Node):
    """Node for calculating V_cmd from image data"""

    def __init__(self):
        super().__init__('ibvs_node')

        # Very NB variables
        self.max_lin_vel = 0.25 * 0.5
        self.max_vert_vel = 0.5
        self.max_angular_vel = 0.15 * 0.3

        self.gain = 0.1

        # QoS policy for ArUco publisher
        qos_aruco = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            depth = 1
        )

        # Collect camera information
        self.camera_info = None
        self.get_camera_info()

        # Most temporary of temporary variables here
        # Hehe.
        # Value for static landing = 305
        # Value for dynamic following = 250
        self.out_offset = 200
        self.in_offset = self.out_offset * 1.80
        self.Z = 2.5

        # Subscribe to identified image coordinate points
        self.aruco_detcs_subscriber = self.create_subscription(
            ArucoDetcs, "/aruco_detcs", self.aruco_callback, qos_aruco
        )

        # Subscribe to mocap data to (cheatily) get Z-height information
        #self.odometry_subscriber = self.create_subscription(
        #   Odometry, "/model/x500_mono_cam_down_0/odometry", self.odometry_callback, qos_aruco
        #)

        # Create velocity command publisher using custom message
        self.vel_cmd_publisher = self.create_publisher(
            VelIBVS, "/ibvs", 10
        )

    def populate_err(self, msg: VelIBVS, tag: str, err):
        if tag == 'outer':
            msg.outer_err.p1_err.x = err[0][0]
            msg.outer_err.p1_err.y = err[0][1]
        
            msg.outer_err.p2_err.x = err[1][0]
            msg.outer_err.p2_err.y = err[1][1]

            msg.outer_err.p3_err.x = err[2][0]
            msg.outer_err.p3_err.y = err[2][1]
        
            msg.outer_err.p4_err.x = err[3][0]
            msg.outer_err.p4_err.y = err[3][1]

        if tag == 'inner':
            msg.inner_err.p1_err.x = err[0][0]
            msg.inner_err.p1_err.y = err[0][1]
        
            msg.inner_err.p2_err.x = err[1][0]
            msg.inner_err.p2_err.y = err[1][1]

            msg.inner_err.p3_err.x = err[2][0]
            msg.inner_err.p3_err.y = err[2][1]
        
            msg.inner_err.p4_err.x = err[3][0]
            msg.inner_err.p4_err.y = err[3][1]

        return msg

    def get_camera_info(self):
        """ Get camera_info msg and destroy subscriber for neatness """
        # Create camera info message
        self.get_logger().info("Getting camera info")
        self.camera_info = None

        # Create camera info subscriber
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo, "/camera_info", self.camera_info_callback, 10
        )
        sleep(3)

    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().info("Camera info callback triggered")
            self.get_logger().info(f"Camera intrinsics {msg.k}")
            self.camera_info = msg
            self.destroy_subscription(self.camera_info_subscriber)
            self.get_logger().info("Destroying /camera_info subscription")

    def odometry_callback(self, msg: Odometry):
        """ Callback to set Z-height of quad to investigate if that affects IBVS performance """
        self.get_logger().info("Received an odometry callback", once=True)
        self.get_logger().info(f"Current height: {msg.pose.pose.position.z}", once=True)
        self.Z = msg.pose.pose.position.z

    def aruco_callback(self, msg: ArucoDetcs):
        
        self.aruco_detcs = msg
        vel_cmd = VelIBVS()

        # If outer tag is detected...
        if (msg.outer_tag_detc == True):
            v_out, err = self.ibvs_function(self.aruco_detcs.outer_tag, self.out_offset, self.camera_info)
            vel_cmd.outer_velocity.vx   = v_out[0]
            vel_cmd.outer_velocity.vy   = v_out[1]
            vel_cmd.outer_velocity.vz   = v_out[2]
            vel_cmd.outer_velocity.vyaw = v_out[3]

            vel_cmd = self.populate_err(msg = vel_cmd, tag = 'outer', err = err)

        else:
            vel_cmd.outer_velocity.vx   = np.nan
            vel_cmd.outer_velocity.vy   = np.nan
            vel_cmd.outer_velocity.vz   = np.nan
            vel_cmd.outer_velocity.vyaw = np.nan
        
        # If inner tag is detected...
        if (msg.inner_tag_detc == True):
            v_out, err = self.ibvs_function(self.aruco_detcs.inner_tag, self.in_offset, self.camera_info)
            vel_cmd.inner_velocity.vx   = v_out[0]
            vel_cmd.inner_velocity.vy   = v_out[1]
            vel_cmd.inner_velocity.vz   = v_out[2]
            vel_cmd.inner_velocity.vyaw = v_out[3]
            
            vel_cmd = self.populate_err(msg = vel_cmd, tag = 'inner', err = err)
            
        else:
            vel_cmd.inner_velocity.vx   = np.nan
            vel_cmd.inner_velocity.vy   = np.nan
            vel_cmd.inner_velocity.vz   = np.nan
            vel_cmd.inner_velocity.vyaw = np.nan

        # Publish VelCmd message
        self.vel_cmd_publisher.publish(vel_cmd)

    def ibvs_function(self, tag: ArucoTag, offset: int, camera_info: CameraInfo):
        """
        Function to find velocity CMD from pixel error between image plane coordinates of detected ArUco tag corners, and defined desired corner positions.
        Does not command roll or pitch as theses are not directly controlled by PX4

        Parameters:

        Returns:
        v_cmd                         : Velocity command in format [vx_cmd, vy_cmd, vz_cmd, vyaw_cmd]
        """

        """
        TODO: Ideally the orientation shouldnt matter, to decrease unnecessray yawing of the quadrotor - when first identified, run a routine to find the orientation of the goal points
        that minimizes the error, and then lock in that orientation as permanent
        For now, I am lazy

        TODO: Some better way of estimating the height to the target
        """

        p_arr = [tag.p1, tag.p2, tag.p3, tag.p4]

        if p_arr is not None:
            # Extract image properties
            height = camera_info.height
            width = camera_info.width

            # Camera intrinsics
            K = camera_info.k
            f = K[0]

            # Distance to target
            # self.get_logger().info(f"Height: {self.Z}")
            Z = self.Z # Improve this estimate! (sorta done?)

            # Create guide points (for now)
            u0 = round(width/2)
            v0 = round(height/2)

            # Create goal points from image center and offset
            goal_points = np.array([[-offset, +offset],
                                    [-offset, -offset],
                                    [+offset, -offset],
                                    [+offset, +offset]]) + np.array([u0, v0])

            curr_points = np.array([[p_arr[0].x, p_arr[0].y],
                                    [p_arr[1].x, p_arr[1].y],
                                    [p_arr[2].x, p_arr[2].y],
                                    [p_arr[3].x, p_arr[3].y]])

            # Create image Jacobian 
            J = np.zeros([2*4, 4])

            for i in range(1,4+1):

                u = p_arr[i-1].x - u0
                v = p_arr[i-1].y - v0

                # First row of Jacobian for given point
                J[2*(i-1)] = [-f/Z, 0, u/Z, v]

                # Second row of jacobian for given point
                J[2*(i-1) + 1] = [0, -f/Z, v/Z, -u]

            # Calculate commanded velocity based on image error
            err = goal_points - curr_points

            # Need to reformat err matrix into single vector using flatten command
            v = np.matmul(pinv(J), err.flatten())
            v = np.matmul(self.gain * np.eye(4,4), v)

            # Saturate velocity commands
            v = np.array(
                [np.clip(v[0], -self.max_lin_vel, self.max_lin_vel),
                np.clip(v[1], -self.max_lin_vel, self.max_lin_vel),
                np.clip(v[2], -self.max_vert_vel, self.max_vert_vel),
                np.clip(v[3], -self.max_angular_vel, self.max_angular_vel)]
                )

            # self.get_logger().info(f"Publishing velocity cmd: {np.array(v).round(4)}")
            return v, err
        else:
            self.get_logger().info("Tried to run IBVS without set tag identified!")
            v = [np.nan, np.nan, np.nan, np.nan]
            return v, err

def main(args=None):
    print("Starting IBVS node")
    rclpy.init(args=args)
    ibvs_node = IBVSNode()
    rclpy.spin(ibvs_node)
    ibvs_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)