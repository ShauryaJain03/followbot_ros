from setuptools import find_packages, setup
import os 
from glob import glob
package_name = 'bot_modes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shaurya',
    maintainer_email='jainshaurya.sj@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "follow_node = bot_modes.follow_node:main",
            "return_node = bot_modes.return_node:main",
            "gps_waypoint_node = bot_modes.gps_waypoint_node:main"
        ],
    },
)


