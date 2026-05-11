import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bot_description = get_package_share_directory("bot_description")
    lidarslam = get_package_share_directory("lidarslam")

    main_param_dir_default = os.path.join(
        bot_description, "config", "lidarslam_baylands.yaml"
    )
    save_dir_default = "/tmp/lidarslam_followbot"

    return LaunchDescription([
        DeclareLaunchArgument(
            "main_param_dir",
            default_value=main_param_dir_default,
            description="Full path to the lidarslam parameter YAML file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz for lidarslam.",
        ),
        DeclareLaunchArgument(
            "use_graph_based_slam",
            default_value="true",
            description="Start graph_based_slam backend.",
        ),
        DeclareLaunchArgument(
            "input_cloud",
            default_value="/velodyne_points",
            description="Input cloud topic.",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/imu_raw",
            description="IMU topic. Kept remapped even when use_imu is false.",
        ),
        DeclareLaunchArgument(
            "save_dir",
            default_value=save_dir_default,
            description="Directory used by graph_based_slam when saving maps.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidarslam, "launch", "lidarslam.launch.py")
            ),
            launch_arguments={
                "main_param_dir": LaunchConfiguration("main_param_dir"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "global_frame_id": "map",
                "robot_frame_id": "base_link",
                "odom_frame_id": "odom",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_graph_based_slam": LaunchConfiguration("use_graph_based_slam"),
                "input_cloud": LaunchConfiguration("input_cloud"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "save_dir": LaunchConfiguration("save_dir"),
                "base_frame": "base_link",
                "lidar_frame": "laser_link",
                "publish_static_tf": "false",
                "use_odom_input": "false",
            }.items(),
        ),
    ])
