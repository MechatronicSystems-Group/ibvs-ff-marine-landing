from launch import LaunchDescription
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
                {'use_sim_time': True},
            ]
        ),
    ])