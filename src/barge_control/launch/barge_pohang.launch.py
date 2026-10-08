from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('section', default_value='full'),
        Node(
            name="barge_pohang_cmd_node",
            package="barge_control",
            executable="barge_pohang_node",
            parameters=[
                {'section': LaunchConfiguration('section')},
                {'use_sim_time': True}
            ]
        )
    ])