from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_detector',
            executable='apriltag_detector_node',
            name='apriltag_detector',
            output='screen',
            parameters=[
                {'image_transport': 'raw'},
                {'tag_family': 'tf36h11'}  
            ],
            remappings=[
                ('image', '/camera/image_raw'),           
                ('camera_info', '/camera/camera_info'),   
                ('tags', '/apriltag_detections')
            ]
        ),
        Node(
            package="apriltag_navigation",
            executable="apriltag_navigation_node",
            name="apriltag_logger",
            output="screen"
        )

    ])
