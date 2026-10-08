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
                {'search_height' : -2.4},
                {'ogain' : [.10, .10, .20, .40]},
                {'igain' : [.20, .20, .15, .40]},
		        {'itag_offset' : 115},
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
