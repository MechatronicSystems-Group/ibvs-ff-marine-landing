from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_groundtruth_bridge',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
            arguments=[
                '/tag/otag/gz_groundtruth_gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                '/tag/itag/gz_groundtruth_gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                '/barge/gz_groundtruth_gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            ]
        ),

        # Nodes to map groundtruth GPS to PX4 local frame
        Node(
            name="barge_gps_groundtruth_node",
            package="utils",
            executable="generic_gps_transformer",
            parameters=[
                {"gps_ros_topic": "/barge/gz_groundtruth_gps"},
                {'use_sim_time': True}
            ]
        ),

        Node(
            name="otag_gps_groundtruth_node",
            package="utils",
            executable="generic_gps_transformer",
            parameters=[
                {"gps_ros_topic": "/tag/otag/gz_groundtruth_gps"},
                {'use_sim_time': True}
            ]
        ),

        Node(
            name="itag_gps_groundtruth_node",
            package="utils",
            executable="generic_gps_transformer",
            parameters=[
                {"gps_ros_topic": "/tag/itag/gz_groundtruth_gps"},
                {'use_sim_time': True}
            ]
        ),

    ])