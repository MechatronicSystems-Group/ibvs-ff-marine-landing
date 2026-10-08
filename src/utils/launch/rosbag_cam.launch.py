import launch

topics = [
    '/x500/camera/image',
]

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record'] + topics,
            output='screen'
        )
    ])
