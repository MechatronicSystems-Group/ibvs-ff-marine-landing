from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        # Launch MixroXRCE agent to bridge PX4 and ROS2
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/cp2102px4'],
            name='px4_serial_bridge',
            output='screen'
        ),

    ])
