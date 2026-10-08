import os
import math
import rclpy
import numpy as np
import pandas as pd
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point, Quaternion
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from time import sleep
import gz.transport13

class WorldControl(gz.transport13.Node):
    def __init__(self):
        super().__init__()

    def update_model_position(self, model, x, y, z):
        pose = Pose()
        pose.name = model
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)

        timeout = 200

        self.request("world/pohang_canal_large/set_pose", pose, request_type = Pose, response_type = Boolean, timeout = timeout) 

def quat2eul(q):
        """
        One would think there would be a library for this sigh
        """
        w = q[3]
        x = q[0]
        y = q[1]
        z = q[2]

        roll = math.atan2(2.0*(y*z + w*x), w*w - x*x - y*y + z*z)
        pitch = math.asin(-2.0*(x*z - w*y))
        yaw = math.atan2(2.0*(x*y + w*z), w*w + x*x - y*y - z*z)

        return [roll, pitch, yaw]

def eul2quat(roll, pitch, yaw):
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x, y, z, w]

def quaternion_multiply(q1, q2):
    """Quaternion multiplication: q = q1 * q2."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return np.array([x, y, z, w])

def slerp(q0, q1, t):
    """Spherical linear interpolation between two quaternions."""
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        result = q0 + t*(q1 - q0)
        return result / np.linalg.norm(result)
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    sin_theta = np.sin(theta)
    s0 = np.sin(theta_0 - theta) / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return s0*q0 + s1*q1

class BargeControllerNode(Node):
    def __init__(self):

        super().__init__('barge_controller_node')

        # QoS Policy
        qos = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        self.origin_x = 0
        self.origin_y = 0
        self.origin_z = 2

        # Create an odometry publisher to publish trajectories to the platform in Gazebo
        self.odometry_publisher = self.create_publisher(Odometry, "/barge/cmd", qos)

        # Load the position data
        has_data = False

        # If it is available here:
        try:
            df = pd.read_csv("baseline.csv")
            has_data = True
        except:
            self.get_logger().info("Baseline.csv not available in immediate folder, trying resources...")

        if not has_data:
            try:
                share_dir = get_package_share_directory('barge_control')
                csv_path = os.path.join(share_dir, 'resource', 'baseline.csv')
                df = pd.read_csv(csv_path)
            except:
                self.get_logger().error("Baseline.csv not able to be loaded.")

        self.timestamps = df['unix_time_offset'].to_numpy()
        self.positions = df[['x_norm', 'y_norm', 'z']].to_numpy()
        self.orientations = df[['qx', 'qy', 'qz', 'qw']].to_numpy()

        # np.pi is exactly opposite
        # ...np.pi is also opposite what the fuck
        theta = 2*np.pi # bizarre
        q_offset = np.array([0.0, 0.0, np.sin(theta/2), np.cos(theta/2)])

        for i in range(len(self.orientations)):
            rpy = quat2eul(self.orientations[i])
            rpy[2] = -rpy[2]

            quat = np.array(eul2quat(rpy[0], rpy[1], rpy[2]))

            self.orientations[i] = quat
            self.orientations[i] = quaternion_multiply(q_offset, self.orientations[i])

        # Section of barge to simulate
        self.declare_parameter('section', 'canal')
        self.section = self.get_parameter('section').get_parameter_value().string_value

        # Slowdown factor
        self.declare_parameter('slowdown_factor', 1.0)
        self.slowdown_factor = self.get_parameter('slowdown_factor').value

        self.world_node = WorldControl()

        sleep(5) # Wait for things to settle

        self.sim_ready = False

        self.clock_sub = self.create_subscription(
            Clock, "/clock", self.clock_callback, 50
        )

    def clock_callback(self, msg):
        if not self.sim_ready:
            self.sim_ready = True
            self.get_logger().info("Simulation clock detected. Starting trajectory timer.")

            # Try to calm down unchill PI controller
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'

            msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            data_index = 0
            delay = 0.5

            match self.section:
                case 'full':
                    data_index = 0
                    msg.pose.pose.position = Point(x=0.0, y=0.0, z=1.9)
                    self.world_node.update_model_position("pohang_barge", 0.0, 0.0, 1.9)
                    self.odometry_publisher.publish(msg)
                    sleep(delay)
                    self.world_node.update_model_position("x500_custom_0", -10.0, -10.0, 5.5)

                case 'canal':
                    data_index = 434
                    msg.pose.pose.position = Point(x=-109.91, y=141.88, z=1.9)
                    self.world_node.update_model_position("pohang_barge", -109.91, 141.88, 1.9)
                    self.odometry_publisher.publish(msg)
                    sleep(delay)
                    self.world_node.update_model_position("x500_custom_0", -98.66, 150.28, 5.5)

                case 'port':
                    data_index = 3255
                    msg.pose.pose.position = Point(x=-636.0, y=1770.0, z=1.9)
                    self.world_node.update_model_position("pohang_barge", -636.00, 1770.00, 1.9)
                    self.odometry_publisher.publish(msg)
                    sleep(delay)
                    self.world_node.update_model_position("x500_custom_0",-626.00, 1780.00, 5.5)

                case 'near_coastal':
                    data_index = 7153 # ~7153
                    msg.pose.pose.position = Point(x=496.0, y=2291.0, z=1.9)
                    self.world_node.update_model_position("pohang_barge", 496.00, 2291.00, 1.9)
                    self.odometry_publisher.publish(msg)
                    sleep(delay)
                    self.world_node.update_model_position("x500_custom_0", 506, 2301, 5.5)

            self.start_wall_time = 0.0
            self.start_data_time = self.timestamps[data_index] # Can select other parts of data to start with

            # Create timer to handle publishing of setpoints to Gazebo
            # 10 Hz seems reasonable
            self.timer = self.create_timer(1.0/10.0, self.timer_callback)


    def timer_callback(self):
        
        self.start_wall_time = self.start_wall_time + 0.10
        target_time = self.start_data_time + self.start_wall_time / self.slowdown_factor

        idx = np.searchsorted(self.timestamps, target_time)
        
        if idx == 0:
            pos = self.positions[0]
        
        else:
            t0 = self.timestamps[idx - 1]
            t1 = self.timestamps[idx]
            alpha = (target_time - t0) / (t1 - t0)

            pos0 = self.positions[idx - 1]
            pos1 = self.positions[idx]
            pos = pos0 + alpha * (pos1 - pos0)

            q0 = self.orientations[idx - 1]
            q1 = self.orientations[idx]
            ori = slerp(q0, q1, alpha)

        print(f"{self.start_wall_time:.1f}, {pos}")

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position = Point(x=pos[1], y=pos[0], z=pos[2] - 1.35)
        msg.pose.pose.orientation = Quaternion(x=ori[0], y=ori[1], z=ori[2], w=ori[3])

        self.odometry_publisher.publish(msg)

def main(args=None):
    # Define and spin up ROS2 node
    rclpy.init(args=args)
    barge_controller = BargeControllerNode()
    rclpy.spin(barge_controller)

    # Explicitly kill ROS2 node on exit
    barge_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
