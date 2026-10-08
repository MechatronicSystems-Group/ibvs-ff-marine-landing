from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Main feedforward controller node
        Node(
            package='ibvs',
            executable='main',
            name='main_node',
            output='screen',
            parameters=[
                {'search_height' : -10.0},
                {'ogain' : [.12, .12, .20, .40]},
                {'igain' : [.30, .30, .01, .20]},
                {'itag_offset' : 100},
            ]
        ),

        # EKF launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ekf'),
                    'launch',
                    'ekf_staged_real.launch.py'
                ])
            ])
        )

    ])
