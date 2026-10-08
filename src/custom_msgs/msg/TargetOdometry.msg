# Message to describe state of AruCo EKF tracking

# Standard header info
std_msgs/Header header
string child_frame_id

# Tag centre odometry - fused result of GPS, outer and inner tags
# Note that child_frame_id == header.frame_id as twist is in inertial frame
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

float32 mag

bool fusing_otag
bool fusing_itag
float32 filter_conf
