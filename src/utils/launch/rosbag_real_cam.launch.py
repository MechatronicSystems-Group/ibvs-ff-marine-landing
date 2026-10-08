import launch
from launch_ros.actions import Node

# The source topic and the new 5Hz downsampled topic
source_topic = '/x500/camera/image'
throttled_topic = '/x500/camera/image_throttled'
target_rate = '5.0'

topics = [
    '/tag/corners',
    '/tag/itag/camera_aruco_pose_estimate',
    '/tag/itag/gz_groundtruth_gps',
    '/tag/itag/gz_groundtruth_odom',
    '/tag/otag/camera_aruco_pose_estimate',
    '/tag/otag/gz_groundtruth_gps',
    '/tag/otag/gz_groundtruth_odom',
    '/x500/camera/camera_info',
    '/x500/diag/state',
    '/x500/ekf/target_odometry',
    '/x500/ekf/itag_transformed_pose',
    '/x500/ekf/otag_transformed_pose',
    '/barge/cmd',
    '/barge/gps',
    '/barge/gz_groundtruth_gps',
    '/x500/px4_local/barge/gps',
    '/x500/px4_local/barge/gz_groundtruth_gps',
    '/x500/px4_local/tag/itag/gz_groundtruth_gps',
    '/x500/px4_local/tag/otag/gz_groundtruth_gps',
    '/fmu/out/vehicle_local_position',
    '/fmu/out/vehicle_global_position',
    '/fmu/out/vehicle_gps_position',
    '/fmu/out/vehicle_odometry',
    '/fmu/in/vehicle_visual_odometry',
    '/LandingPad/pose_stamped',
    '/X500/pose_stamped',
    throttled_topic
]

def generate_launch_description():
    return launch.LaunchDescription([

        Node(
            package='topic_tools',
            executable='throttle',
            name='camera_image_throttler',
            arguments=['messages', source_topic, target_rate, throttled_topic],
            output='screen'
        ),

        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record'] + topics,
            output='screen'
        )
    ])
