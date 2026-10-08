from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        # Node to map GPS to PX4 local frame
        Node(
            name="barge_gps_px4_local_map_node",
            package="utils",
            executable="generic_gps_transformer",
            parameters=[
                {"gps_ros_topic": "/barge/gps"},
            ]
        ),

        # EKF node
        Node(
            name="staged_target_ekf_node",
            package="ekf",
            executable="target_ekf_staged",
            output='screen',
            parameters=[
                {"use_mocap_visual_odometry": False},
                {"cam_x": -0.0060}, # m
                {"cam_y": -0.0327}, # m
                {"cam_z": 0.1745}, # m
                {"cam_roll": 4.2373}, # deg
                {"cam_pitch": -0.0028}, # deg
                {"cam_yaw": 0.1577}, # deg
            ]
        ),
    ])
