from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        # Launch MixroXRCE agent to bridge PX4 and ROS2
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyUSB0', '-b', '921600'],
            name='px4_udp_bridge',
            output='screen'
        ),

    ])
