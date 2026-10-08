from launch.launch_description import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from pathlib import Path

def generate_launch_description() -> LaunchDescription:

    calib_path = Path(get_package_share_directory('utils')) / 'calib/rpi_cam_calib.yaml'
    calib_url = calib_path.as_uri()

    return LaunchDescription([

        Node(
            name="rpi_global_shutter_cam_node",
            package="camera_ros",
            executable="camera_node",
            parameters=[{
                "camera": 0,
                "width": 960,
                "height": 640,
                "format": "RGB888",
                "camera_info_url": calib_url,

		# QoS overrides
		"qos_overrides./x500/camera/image.publisher.reliability": "best_effort",
    		"qos_overrides./x500/camera/image.publisher.history": "keep_last",
    		"qos_overrides./x500/camera/image.publisher.depth": 5,

    		"qos_overrides./x500/camera/image/compressed.publisher.reliability": "best_effort",
    		"qos_overrides./x500/camera/image/compressed.publisher.depth": 5,

    		"qos_overrides./x500/camera/camera_info.publisher.reliability": "reliable",
    		"qos_overrides./x500/camera/camera_info.publisher.depth": 10,
            }],
            remappings=[
                ("~/image_raw", "/x500/camera/image"),
                ("~/image_raw/compressed", "/x500/camera/image/compressed"),
                ("~/camera_info", "/x500/camera/camera_info"),
            ]
        )
    ])
