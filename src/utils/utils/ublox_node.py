import rclpy
import serial
import struct
import collections
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from time import sleep

class UBloxNode(Node):

    def __init__(self):
        super().__init__("ublox_gps_node")
        self.get_logger().info("Starting ublox GPS node")
        # Init function

        # Declare parameters
        self.declare_parameter("serial_port", "/dev/esp32")
        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value

        self.declare_parameter("baud_rate", 115200)
        baudrate = self.get_parameter("baud_rate").get_parameter_value().integer_value

        # Vague attempt at data validation
        self.previous_tstamp = 0
        self.previous_struct = 0

        # Open connection, retrying each time
        connected = False
        while not connected:
            try:
                self.get_logger().info("Attempting to open serial port")
                self.esp32 = serial.Serial(port=serial_port, baudrate=baudrate, timeout=1)
                connected = True

            except:
                self.get_logger().info("Failed to open serial port with ESP32, retrying...")
                sleep(1)

        self.get_logger().info("Connected to ESP32 GPS receiver")

        # Create publisher for GPS data (real)

        self.declare_parameter("gps_topic", "barge/gps")
        pub_topic = self.get_parameter("gps_topic").get_parameter_value().string_value

        qos = QoSProfile(            
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.VOLATILE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1
        )

        self.gps_pub = self.create_publisher(
            NavSatFix, pub_topic, qos
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        
        FMT = '<IiiibB'
        packetSize = struct.calcsize(FMT)
        
        # New data (packet) available
        if (self.esp32.in_waiting >= packetSize):

            raw_data = self.esp32.read(packetSize)
            data = struct.unpack(FMT, raw_data)

            # Populate msg
            navmsg = NavSatFix()
            navmsg.header.stamp = self.get_clock().now().to_msg()
            navmsg.header.frame_id = "barge"
            navmsg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            # Convert and populate data
            navmsg.latitude = float(data[1]) / 1e7      # scale to degrees 
            navmsg.longitude = float(data[2]) / 1e7     # scale to degrees
            navmsg.altitude = float(data[3]) / 1e3      # scale to meters
            
            # Set covariance correctly
            if (data[4] == 0): # Yes fix
                navmsg.status.status = NavSatStatus.STATUS_FIX
            else: # no fix
                navmsg.status.status = NavSatStatus.STATUS_NO_FIX

            self.gps_pub.publish(navmsg)

def main():
    rclpy.init()
    ublox_node = UBloxNode()
    rclpy.spin(ublox_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
