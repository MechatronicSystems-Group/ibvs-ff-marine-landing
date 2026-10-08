import launch
from launch_ros.actions import Node

# The source topic and the new 5Hz downsampled topic
source_topic = '/x500/camera/image'
throttled_topic = '/x500/camera/image_throttled'
target_rate = '5.0'

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
            cmd=['ros2', 'bag', 'record', throttled_topic],
            output='screen'
        )
    ])
