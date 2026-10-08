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
                {'use_sim_time': True}
            ]
        ),

        # EKF node
        Node(
            name="staged_target_ekf_node",
            package="ekf",
            executable="target_ekf_staged",
            output='screen',
            parameters=[
                {"filter_propogate": True},
                {'use_sim_time': True}
            ]
        ),
    ])