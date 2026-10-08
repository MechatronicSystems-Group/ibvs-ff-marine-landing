from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

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

    # Items without substitutions
    micro_xrce = get_launch_no_args('px4_ros2_launch', 'microxrce.launch.py')

    aruco = get_launch_no_args('aruco', 'aruco_pohang.launch.py')

    barge_bridge = get_launch_no_args('barge_control', 'barge_bridge.launch.py')

    gz_groundtruth = get_launch_no_args('utils', 'gz_groundtruth.launch.py')

    ekf = get_launch_no_args('ekf', 'ekf_staged.launch.py')

    # PX4 is more complex
    px4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('px4_ros2_launch'),
                'launch',
                'px4.launch.py'
            ])
        ]),
        launch_arguments={
            'px4_gz_world': 'pohang_canal_large',
            'px4_sim_model': 'x500_custom',
            'px4_gz_model_pose': '0,0,10.0,0,0,0'
        }.items()
    )

    ld = LaunchDescription([
        micro_xrce,
        aruco,
        barge_bridge,
        ekf,
        px4,
        gz_groundtruth,
    ])

    return ld
