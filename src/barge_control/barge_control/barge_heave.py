import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

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
        self.origin_z = 1

        # Create an odometry publisher to publish trajectories to the platform in Gazebo
        # (or maybe the real Stewart Platform in time...)
        self.odometry_publisher = self.create_publisher(Odometry, "/barge_cmd", qos)

        # Create timer to handle publishing of setpoints to Gazebo
        # 100Hz seems reasonable?
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        # Get current time
        current_time = self.get_clock().now()
        
        # Calculate time in seconds since node started (or use a baseline)
        time_sec = current_time.nanoseconds / 1e9
        
        # Calculate heaving motion: amplitude = 0.3m, period = 5s
        amplitude = 0.75 # 30cm
        period = 5 # 5 seconds
        frequency = 2 * math.pi / period
        
        # Calculate Z position using sine wave
        z_position = amplitude * math.sin(frequency * time_sec)
        
        # Create odometry message
        odom_msg = Odometry()
        
        # Set header
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # Set position (heaving in Z)
        odom_msg.pose.pose.position.x = 0.0 + self.origin_x
        odom_msg.pose.pose.position.y = 0.0 + self.origin_y
        odom_msg.pose.pose.position.z = z_position + self.origin_z
        
        # Set orientation (no rotation)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0
        
        # Calculate Z velocity (derivative of position)
        z_velocity = amplitude * frequency * math.cos(frequency * time_sec)
        
        # Set twist (velocity)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = z_velocity
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        
        # Publish the message
        self.odometry_publisher.publish(odom_msg)

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
