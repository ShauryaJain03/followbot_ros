import os
import site

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    elevation_share = get_package_share_directory('elevation_mapping_cupy')
    description_share = get_package_share_directory('bot_description')
    cuda_nvrtc_lib = os.path.join(
        site.getusersitepackages(), 'nvidia', 'cuda_nvrtc', 'lib')
    cuda_library_path = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        cuda_nvrtc_lib + ':' + os.environ.get('LD_LIBRARY_PATH', ''),
    )

    return LaunchDescription([
        cuda_library_path,
        Node(
            package='elevation_mapping_cupy',
            executable='elevation_mapping_node.py',
            name='elevation_mapping_node',
            parameters=[
                os.path.join(elevation_share, 'config', 'core', 'core_param.yaml'),
                os.path.join(
                    description_share, 'config', 'followbot_elevation_mapping.yaml'),
                {'use_sim_time': True},
            ],
            output='screen',
        ),
        Node(
            package='bot_bringup',
            executable='analytic_traversability',
            name='analytic_traversability',
            parameters=[
                os.path.join(
                    description_share, 'config', 'followbot_traversability.yaml'),
                {'use_sim_time': True},
            ],
            output='screen',
        ),
    ])
