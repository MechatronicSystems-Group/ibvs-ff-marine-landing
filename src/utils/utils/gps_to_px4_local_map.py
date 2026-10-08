import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from px4_msgs.msg import VehicleLocalPosition
import pymap3d as pm

class GPStoLocal(Node):
    def __init__(self):
        super().__init__("gps_to_local_node")

        self.qos_px4 = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        self.declare_parameter('gps_ros_topic', "empty_gps_topic")

        # Vehicle odometry subscriber
        self.vehicle_home_pos_subscriber = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.home_pos_callback, self.qos_px4
        )
        # Vehicle odometry subscriber (backup)
        self.vehicle_home_pos_subscriber = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.home_pos_callback, self.qos_px4
        )
        self.home_gps_coords = None

        self.topic = self.get_parameter('gps_ros_topic').get_parameter_value().string_value

        if (self.topic[0] == '/'):
            self.get_logger().info("Removing leading '/' to avoid err.")
            self.topic = self.topic[1:]

        if self.topic == "empty_gps_topic":
            self.get_logger().error(f"Empty GPS topic given to gps_to_px4_local_map for topic: {self.topic}")
        else:
            # GPS subscription
            self.gps_subscription = self.create_subscription(
                NavSatFix, 
                self.topic,
                self.gps_callback,
                self.qos_px4
            )

        # Create publisher
        self.pub_topic = "x500/px4_local/" + self.topic
        self.pose_publisher = self.create_publisher(
            PoseStamped, self.pub_topic, self.qos_px4
        )

        

    def home_pos_callback(self, msg: VehicleLocalPosition):
        self.home_gps_coords = msg

    def gps_callback(self, msg: NavSatFix):
        
        if isinstance(self.home_gps_coords, VehicleLocalPosition):
            
            self.get_logger().info(f"Starting gps to px4_local map: {self.topic} -> {self.pub_topic}", once=True)

            # Convert to local position coordinates
            n, e, d = pm.geodetic2enu(
                lat=msg.latitude,
                lon=msg.longitude,
                h=msg.altitude,
                lat0=self.home_gps_coords.ref_lat,
                lon0=self.home_gps_coords.ref_lon,
                h0=self.home_gps_coords.ref_alt
            )

            px4_local = PoseStamped()
            px4_local.header.stamp = self.get_clock().now().to_msg()
            px4_local.header.frame_id = "px4_local"

            # n,e,d data to x,y,z in ENU px4_local
            px4_local.pose.position.x = e
            px4_local.pose.position.y = n
            px4_local.pose.position.z = d
    
            self.pose_publisher.publish(px4_local)

        else:
            self.get_logger().info(f"Waiting for NavSatFix for GPS map node on topic: {self.topic}", throttle_duration_sec = 3.0)



def main():
    rclpy.init()
    node = GPStoLocal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()