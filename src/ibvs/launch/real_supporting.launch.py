from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def get_launch_no_args(package, launch_file):

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(package),
                'launch',
                launch_file
            ])
        ])
    )

def generate_launch_description():

    # Packages to launch without substitutions
    micro_xrce = get_launch_no_args('px4_ros2_launch', 'microxrce_real.launch.py')
    
    camera = get_launch_no_args('utils', 'rpi_global_cam.launch.py')

    aruco = get_launch_no_args('aruco', 'aruco_real.launch.py')

    ld = LaunchDescription([
        micro_xrce,
        camera,
        aruco
    ])

    return ld