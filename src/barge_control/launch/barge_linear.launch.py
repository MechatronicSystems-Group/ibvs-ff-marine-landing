from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            name="barge_linear_cmd_node",
            package="barge_control",
            executable="barge_linear_node",
            parameters=[
                {'use_sim_time': True}
            ]
        )
    ])