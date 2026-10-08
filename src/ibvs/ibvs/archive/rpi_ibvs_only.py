from arucoquad import ArucoQuad, Tag
from custom_msgs.msg import TagCorners
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import rclpy

class X500(ArucoQuad):

    def __init__(self):
        # Init. functionality of various subclasses
        super().__init__()

        # Velocity tracking variables
        self.ibvs_cmd = np.zeros([4], dtype=np.float32)
        self.ff_cmd = np.zeros([4], dtype=np.float32)
        self.vel_cmd = np.zeros([4], dtype=np.float32)

        # IBVS objects, functions and booleans
        self.otag = Tag()
        self.itag = Tag()

        self.ogain = [.2, .2,  .4, .4] # IBVS outer tag gain
        self.igain = [.5, .5,  .6, .4] # IBVS inner tag gain
        self.lgain = [.6, .6, 1.0, .2] # IBVS inner tag landing gain
        self.set_camera_parameters() # Set camera parameters

        # Subscribe to ArucoDetcs
        self.aruco_detcs_subscriber = self.create_subscription(
            TagCorners, "/tag/corners", self.aruco_callback, QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT, 
                durability=DurabilityPolicy.VOLATILE, 
                history=HistoryPolicy.KEEP_ALL, 
                depth=1
            )
        )

        # State timer
        self.state_machine_hz = 50
        self.state_timer = self.create_timer(1/self.state_machine_hz, self.state_timer_callback)

    def aruco_callback(self, msg: TagCorners):

        # Only run if tag detected:
        if msg.otag:
            self.otag.tracked = True

            # Update otag * raw points *
            self.otag.p = [msg.otag_corners[0], msg.otag_corners[1], msg.otag_corners[2], msg.otag_corners[3]]

            # Update otag * virtual points *
            self.otag.pv = self.map_to_virtual(self.otag.p)
            
        else:
            self.otag.tracked = False

        if msg.itag:
            self.itag.tracked = True

            # Update itag * raw points *
            self.itag.p = [msg.itag_corners[0], msg.itag_corners[1], msg.itag_corners[2], msg.itag_corners[3]]

            # Update itag * virtual points *
            self.itag.pv = self.map_to_virtual(self.itag.p)

        else:
            self.itag.tracked = False
    
    def state_timer_callback(self):
        
        if (self.otag.tracked):
            self.ibvs_cmd_raw, err = self.ibvs_wrapper(points=self.otag.p, offset=135, z=2.5, gain_matrix=self.ogain)
            print(self.ibvs_cmd_raw)


    def ibvs_wrapper(self, points, offset, z, gain_matrix):
        vel_cmd, err, _, _ = self.solve_ibvs(p_arr=points, offset=offset, Z=z, gain=gain_matrix)

        heading = 0.0

        # 2D map
        R = np.array([[np.cos(heading), -np.sin(heading)],
                [np.sin(heading),  np.cos(heading)]], dtype=np.float32)
                
        v_p = R @ np.array([[vel_cmd[0]], 
                            [vel_cmd[1]]])

        # Output vector: vx, vy, vz, vyaw
        v = np.array([v_p[0][0], v_p[1][0], vel_cmd[2], vel_cmd[3]], dtype=np.float32)

        return v, err

def main(args=None) -> None:

    # Spin up ROS2
    rclpy.init(args=args)

    x500_rpi_test_node = X500()

    try:
        rclpy.spin(x500_rpi_test_node)
    except SystemExit:
        x500_rpi_test_node.destroy_node()

if __name__ == '__main__':
    main()
