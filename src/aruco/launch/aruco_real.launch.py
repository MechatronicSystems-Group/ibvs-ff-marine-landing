from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco',
            executable='aruco_node',
            name='aruco_id_service',
            output='screen',
            parameters=[
                {'image_topic' : '/x500/camera/image'},
                {'camera_info_topic' : '/x500/camera/camera_info'},
                {'otag_size' : 0.495},
                {'itag_size' : 0.095},
            ]
        ),
    ])
