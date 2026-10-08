from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from px4_msgs.msg import VehicleAttitude
from scipy.spatial.transform import Rotation as R
from quad import Quad
import numpy as np
import math

class ArucoQuad(Quad):
    """
    Extends the Quad class with easy creation of callback functions for /aruco_detcs or other image/vision systems.
    Not sure anyone else will ever read any of this but alas, here I am. Writing comments.
    """

    def __init__(self, node_name = 'arucoQuad'):
        """
        ArucoQuad init function
        """
        super().__init__(node_name)

        # Disable until CameraInfo message received (this can be conceivably hardcoded with a known camera)
        self.camera_info_received = False

        # Saturation values (move these to a config file?)
        self.max_lin_vel = 1.5
        self.max_vert_vel = 1.5
        self.max_angular_vel = 0.5
        
    def set_camera_parameters(self, height, width, f):
        """
        Sets camera parameters for use in IBVS.
        """
        self.height = height
        self.width = width
        self.f = f
        self.camera_info_received = True
        self.img_centre = np.array([self.width/2, self.height/2], dtype=np.float32)

    def camera_param_callback(self, msg: CameraInfo):
        """
        Callback triggered by camera param subscriber set in main file. Auto sets camera parameters for IBVS and updates
        self.camera_info_received variable
        """
        if not self.camera_info_received:
            self.get_logger().info(f"Setting camera info from callback. Height: {msg.height}, Width: {msg.width}, f: {msg.k[0]}")
            self.set_camera_parameters(height=msg.height, width=msg.width, f=msg.k[0])
            self.camera_info_received = True

    def solve_ibvs(self, p_arr, offset, Z, gain, ad_z = 1.0):

        if p_arr is not None:
            
            # Camera + image properties
            height = self.height
            width = self.width
            f = self.f

            # Create guide point centers
            u0 = round(width/2)
            v0 = round(height/2)

            # Create goal points from image center and offset
            goal_points = np.array([[-offset, +offset],
                                    [-offset, -offset],
                                    [+offset, -offset],
                                    [+offset, +offset]]) + np.array([u0, v0])

            curr_points = np.array([[p.x, p.y] for p in p_arr])

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
            v = np.matmul(np.linalg.pinv(J), err.flatten())
            v[2] *= ad_z # Apply adaptive gain to vertical velocity component based on Cho et al. 2022 
            v = gain * v
            
            # Saturate velocity commands
            v = np.array(
                [np.clip(v[0], -self.max_lin_vel, self.max_lin_vel),
                np.clip(v[1], -self.max_lin_vel, self.max_lin_vel),
                np.clip(v[2], -self.max_vert_vel, self.max_vert_vel),
                np.clip(v[3], -self.max_angular_vel, self.max_angular_vel)]
                )
            
            # Goal points to array of ROS2 Point() classes
            goal_points = [Point(x=float(goal_points[i][0]), y=float(goal_points[i][1])) for i in range(4)]
            return v, err, goal_points, J
        
        else:
            raise Exception("Tried to run IBVS without populated tag message")

    def eul2quat(self, roll, pitch, yaw):
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

        return [w, x, y, z]

    def map_to_virtual(self, points, width = 1280, height = 960, f = 539.936):
        """
        Casts tags from a given ArucoDetcs message to the virtual plane based on the roll and pitch of the quadrotor.
        """

        # Arrays
        pv = [Point(), Point(), Point(), Point()]

        # Process outer tag
        if self.vehicle_attitude is not None:

            # Offsets
            u_offset = width/2
            v_offset = height/2

            r_uav = R.from_quat(self.get_vehicle_attitude().q, scalar_first=True)
            [_, pitch, roll] = r_uav.as_euler('zyx')

            pitch = pitch
            roll = roll

            for i in range(0,4):
                p = points[i]
                u = p.x - u_offset
                v = p.y - v_offset

                # Map u and v to virtual image plane
                denom = (-u*math.cos(roll)*math.sin(pitch) + v*math.sin(roll) + f*math.cos(roll)*math.cos(pitch))

                pv[i].x = round((f * (u*math.cos(pitch) + f*math.sin(pitch)) / denom) + u_offset, 2)
                pv[i].y = round((f * (u*math.sin(roll)*math.sin(pitch) + v*math.cos(roll) - f*math.sin(roll)*math.cos(pitch)) / denom) + v_offset, 2)

        elif self.vehicle_attitude is None:
            #self.info("Attempted to process outer tag without vehicle attitude!")
            pass
        
        return pv

    def square_fit(self, points):
        """
        Function to fit a square to the four points of a detected tag, and return the closest-fit
        square points. Used to account for perspective distortion in the image plane.

        Adapted from:
        https://math.stackexchange.com/questions/4598602/optimal-fit-of-four-points-to-a-square/4598802
        """
        
        # Convert points to numpy array
        x = np.array([point.x for point in points], dtype=np.float32)
        y = np.array([point.y for point in points], dtype=np.float32)

        # find centroid
        x0 = np.mean(x)
        y0 = np.mean(y)

        # get rho
        rho = 1/4 * np.sqrt((x[1] - x[3] + y[2] - y[0])**2 + (x[0] - x[2] + y[1] - y[3])**2)

        # get theta
        denom = np.sqrt((x[1] - x[3] + y[2] - y[0])**2 + (x[0] - x[2] + y[1] - y[3])**2)
        theta = -1 * (np.arctan2((x[0] - x[2] + y[1] - y[3]) / denom, -(x[1] - x[3] + y[2] - y[0]) / denom))

        # generate new points array
        rot = np.pi / 2 # adjust factor to align camera correctly
        psq = np.array([Point(), Point(), Point(), Point()])
        psq[0].x, psq[0].y = x0 + rho * np.cos(rot + theta), y0 + rho * np.sin(rot + theta)
        psq[1].x, psq[1].y = x0 + rho * np.cos(rot + theta + np.pi * 1/2), y0 + rho * np.sin(rot + theta + np.pi * 1/2)
        psq[2].x, psq[2].y = x0 + rho * np.cos(rot + theta + np.pi), y0 + rho * np.sin(rot + theta + np.pi)
        psq[3].x, psq[3].y = x0 + rho * np.cos(rot + theta + np.pi * 3/2), y0 + rho * np.sin(rot + theta + np.pi * 3/2)

        # return points in square fit
        return psq
        

class Tag():
    """
    Class for storing information (and confidence) about each AprilTag. State behaviour can then make decisions based on
    information stored in this class that is updated continually.
    """
    def __init__(self, alpha = 0.9, beta = 0.6, tracked = False):
        
        # Was the tag seen in the last callback?
        self.tracked = tracked

        # Detection confidence
        self.confidence = 0
        self.alpha = alpha
        self.id = None

        # Raw points
        self.p = [Point(), Point(), Point(), Point()]

        # Virtual points
        self.pv = [Point(), Point(), Point(), Point()]

        # Error confidence works similarly
        self.err_filtered = 5000
        self.beta = beta

    def update_confidence(self, D: bool):
        """
        D is bool based on whether AprilTag is detected or not.
        """
        D_val = 1 if D else 0
        self.confidence = self.alpha*self.confidence + (1-self.alpha)*D_val

    def is_confident(self, threshold = 0.8):
        """
        Simply returns bool based on input confidence threshold
        """
        return self.confidence > threshold
    
    def update_err_filtered(self, err):
        """
        Updates the filtered error using np.linalg.norm with the Frobenius norm selected.
        Technically a trivially simple Bayes filter, similarly to confidence value
        """
        fro_norm = np.linalg.norm(err, 'fro')
        self.err_filtered = self.beta*self.err_filtered + (1-self.beta)*fro_norm

    def filtered_err_within_tol(self, threshold = 30):
        """
        Simply return a bool based on input error tolerance threshhold
        """
        return self.err_filtered < threshold