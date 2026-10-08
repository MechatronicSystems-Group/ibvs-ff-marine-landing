import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.node import Node

class FakeBargeNode(Node):

    def __init__(self):
        super().__init__("fake_barge_node")
        self.get_logger().info("Starting fake barge node")

        # Init function

        # Create subscriber
        self.mocap_sub = self.create_subscription(
            PoseStamped, "/fakeBarge/pose_stamped", self.mocap_callback, 1
        )
        self.latest_mocap = None

        self.timer = self.create_timer(0.1, self.timer_callback)

        # Create publisher

        self.qos_px4 = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        self.pose_pub = self.create_publisher(
            PoseStamped, "/x500/px4_local/barge/gps", self.qos_px4
        )

    def mocap_callback(self, msg: PoseStamped):
        self.latest_mocap = msg

    def timer_callback(self):
        
        if isinstance(self.latest_mocap, PoseStamped):
            self.get_logger().info("Received landing pad mocap data", once=True)

            # Transform to px4 convention
            pub_msg = PoseStamped()

            pub_msg.header.frame_id = "px4_local"
            pub_msg.header.stamp = self.latest_mocap.header.stamp

            pub_msg.pose.position.x = self.latest_mocap.pose.position.x
            pub_msg.pose.position.y = -self.latest_mocap.pose.position.y
            pub_msg.pose.position.z = -self.latest_mocap.pose.position.z

            pub_msg.pose.orientation.w = self.latest_mocap.pose.orientation.w
            pub_msg.pose.orientation.x = self.latest_mocap.pose.orientation.x
            pub_msg.pose.orientation.y = -self.latest_mocap.pose.orientation.y
            pub_msg.pose.orientation.z = -self.latest_mocap.pose.orientation.z

            self.pose_pub.publish(pub_msg)

def main():
    rclpy.init()
    fake_barge = FakeBargeNode()
    rclpy.spin(fake_barge)
    rclpy.shutdown()

if __name__ == '__main__':
    main()