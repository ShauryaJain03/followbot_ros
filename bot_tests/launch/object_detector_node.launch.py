from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bot_tests',
            executable='object_detector_node',
            name='object_detector_node',
            output='screen',
            parameters=[
                {'input_topic': '/camera/image_raw'},
                {'output_topic': '/camera/yolo_labeled'},
                {'confidence': 0.25},
            ],
        ),
    ])
