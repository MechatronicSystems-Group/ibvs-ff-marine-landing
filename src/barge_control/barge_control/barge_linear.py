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

        self.home_set = False
        self.origin_x = 0
        self.origin_y = 0
        self.origin_z = 1

        # Create an odometry publisher to publish trajectories to the platform in Gazebo
        # (or maybe the real Stewart Platform in time...)
        self.odometry_publisher = self.create_publisher(Odometry, "/barge/cmd", qos)
        self.odometry_subscriber = self.create_subscription(Odometry, "/barge/odom", self.barge_odom_callback, 1)
        self.barge_odom = Odometry()
        self.barge_odom.pose.pose.position.z = -999.0

        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Create timer to handle publishing of setpoints to Gazebo
        # 100Hz seems reasonable?
        self.timer = self.create_timer(0.01, self.timer_callback)

    def barge_odom_callback(self, msg: Odometry):
        self.barge_odom = msg
        if self.home_set == False:
            self.origin_x = msg.pose.pose.position.x
            self.origin_y = msg.pose.pose.position.y
            self.origin_z = msg.pose.pose.position.z
            self.home_set = True
            print("Starting barge moving")

    def timer_callback(self):
        if (self.home_set == True):
            # Get current time
            current_time = self.get_clock().now()
            
            # Calculate time in seconds since node started (or use a baseline)
            time_sec = current_time.nanoseconds / 1e9
            
            elapsed_time = time_sec - self.start_time

            x_velocity = float(-3.25)
            y_velocity = 0
            x_pos = x_velocity * elapsed_time
            y_pos = y_velocity * elapsed_time
            
            # Create odometry message
            odom_msg = Odometry()
            
            # Set header
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            
            # Set position (heaving in Z)
            odom_msg.pose.pose.position.x = y_pos + self.origin_x
            odom_msg.pose.pose.position.y = x_pos + self.origin_y
            odom_msg.pose.pose.position.z = 0.0 + self.origin_z
            
            # Set orientation (no rotation)
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = 0.0
            odom_msg.pose.pose.orientation.w = 1.0
            
            # Set twist (velocity)
            odom_msg.twist.twist.linear.x = float(x_velocity)
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
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
