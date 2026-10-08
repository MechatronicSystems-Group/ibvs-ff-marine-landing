import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # PX4 launch variables

    # px4_dir - where PX4 is installed on local machine
    px4_dir = LaunchConfiguration('px4_dir')
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value=os.path.expanduser('~/PX4-Autopilot')
    )

    # px4_sys_autostart - required
    px4_sys_autostart = LaunchConfiguration('px4_sys_autostart')
    px4_sys_autostart_arg = DeclareLaunchArgument(
        "px4_sys_autostart",
        default_value='4001'
    )

    # px4_gz_world - gz world in which to simulate
    px4_gz_world = LaunchConfiguration('px4_gz_world')
    px4_gz_world_arg = DeclareLaunchArgument(
        "px4_gz_world",
        default_value="default"
    )

    # px4_sim_model - gz airframe
    px4_sim_model = LaunchConfiguration('px4_sim_model')
    px4_sim_model_arg = DeclareLaunchArgument(
        'px4_sim_model', 
        default_value='x500'
    )

    # px4_gz_model_pose - spawn location for px4 model in sim
    px4_gz_model_pose = LaunchConfiguration("px4_gz_model_pose")
    px4_gz_model_pose_arg = DeclareLaunchArgument(
        'px4_gz_model_pose',
        default_value='0,0,0.5,0,0,0'
    )
    
    px4_exec = ExecuteProcess(
        cmd=[os.path.join("build", "px4_sitl_default", "bin", "px4")],
        cwd=px4_dir,
        output='screen',
    )

    ld = LaunchDescription([
        # Launch arguments
        px4_dir_arg,
        px4_sys_autostart_arg,
        px4_gz_world_arg,
        px4_sim_model_arg,
        px4_gz_model_pose_arg,

        # Set environmental variables
        SetEnvironmentVariable('PX4_SYS_AUTOSTART', px4_sys_autostart),
        SetEnvironmentVariable('PX4_GZ_WORLD', px4_gz_world),
        SetEnvironmentVariable('PX4_SIM_MODEL', px4_sim_model),
        SetEnvironmentVariable('PX4_GZ_MODEL_POSE', px4_gz_model_pose),

        # PX4
        px4_exec,
    ])

    return ld
