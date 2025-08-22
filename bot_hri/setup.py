from setuptools import find_packages, setup
import glob as glob
import os
package_name = 'bot_hri'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rule_engine.launch.py']), 

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
            "sensor_conf = bot_hri.sensor_conf:main",
            "apriltag_conf = bot_hri.apriltag_conf:main",
            "rule_engine = bot_hri.rule_engine:main",
            "bt = bot_hri.bt:main",
        ],
    },
)
