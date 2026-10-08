std_msgs/Header header

string state # state machine current state

# State variables (boolean logic)
bool offboard_enabled
bool armed
bool callback_test
bool ekf_test
bool ibvs
bool ekf_ff

# Detected tag
bool otag_detc
bool itag_detc

# IBVS tag values
float32 otag_conf
float32 itag_conf

float32 otag_err_filt
float32 itag_err_filt

float32 otag_factor
float32 itag_factor

float32 ad_z

float32 yaw_err

# IBVS tag & goal image coordinates
geometry_msgs/Point[4] otag_corners
geometry_msgs/Point[4] itag_corners

int32 goal_offset

# Velocity commands to the controller
float32[4] ibvs_v
float32[4] ff_v
float32[4] cmd_v

# PX4 local position
float32 px4_x
float32 px4_y
float32 px4_z

# Mission simulation
float32 loiter_time
float32 land_time
int32 missions_finished
int32 max_missions

float32 target_x
float32 target_y