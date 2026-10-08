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
                {'otag_size' : 0.50},
                {'itag_size' : 0.10},
                {'use_sim_time': True}
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            arguments=[
                '/x500/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo', 
            ],
            remappings=[
                (
                    '/x500/camera/camera_info',
                    '/x500/camera/camera_info'
                )
            ],
            parameters=[
                {'qos_overrides./x500/camera/camera_info.publisher.reliability': 'best_effort'},
                {'use_sim_time': True}
            ],
            output='screen',
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_image_bridge',
            arguments=[
                '/x500/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image', 
            ],
            remappings=[
                (
                    '/x500/camera/image_raw',
                    '/x500/camera/image'
                )
            ],
            parameters=[
                {'qos_overrides./x500/camera/image.publisher.reliability': 'best_effort'},
                {'use_sim_time': True}
            ],
            output='screen',
        )
    ])