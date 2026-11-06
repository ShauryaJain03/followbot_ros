from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    description_share = get_package_share_directory('bot_description')
    segmentation_share = get_package_share_directory('ground_segmentation_ros2')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                segmentation_share + '/launch/ground_segmentation.launch.py'),
            launch_arguments={
                'pointcloud_topic': '/points',
                'imu_topic': '/imu_raw',
                'params_file': description_share + '/config/followbot_gseg3d.yaml',
                'use_sim_time': 'true',
            }.items(),
        ),
    ])
