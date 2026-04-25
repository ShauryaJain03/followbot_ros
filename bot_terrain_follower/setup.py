import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'bot_terrain_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shaurya',
    maintainer_email='jainshaurya.sj@gmail.com',
    description='Capability-aware terrain following proof of concept.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_ground_truth_publisher = bot_terrain_follower.robot_ground_truth_publisher:main',
            'human_pose_publisher = bot_terrain_follower.human_pose_publisher:main',
            'naive_follower = bot_terrain_follower.naive_follower:main',
            'traversability_analyzer = bot_terrain_follower.traversability_analyzer:main',
            'capability_aware_follower = bot_terrain_follower.capability_aware_follower:main',
            'demo_metrics_logger = bot_terrain_follower.demo_metrics_logger:main',
        ],
    },
)
