from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='barge_bridge',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
            arguments=[
                '/barge/cmd@nav_msgs/msg/Odometry]gz.msgs.Odometry',
                '/barge/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/barge/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                '/x500/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ]
        )
    ])