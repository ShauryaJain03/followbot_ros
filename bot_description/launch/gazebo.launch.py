import os
from os import pathsep
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bot_description = get_package_share_directory("bot_description")

    use_sim_time = LaunchConfiguration("use_sim_time")
    model = LaunchConfiguration("model")
    world_name = LaunchConfiguration("world_name")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(bot_description, "urdf", "bot.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty",
        description="World name to load",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )

    model_paths = [
        str(Path(bot_description).parent.resolve()),
        os.path.join(bot_description, "models"),
    ]

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        pathsep.join(model_paths),
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            model,
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    default_world_directory = os.path.join(bot_description, "worlds")

    world_path = PythonExpression([
        "'", default_world_directory, "/' + '", world_name, "' + '.world'"
    ])

    spawn_x = PythonExpression([
        "'-222.0' if '", world_name,
        "' == 'baylands' else '0.0'"
    ])
    spawn_y = PythonExpression([
        "'-68.0' if '", world_name,
        "' == 'baylands' else '0.0'"
    ])
    spawn_z = PythonExpression([
        "'5.4' if '", world_name,
        "' == 'baylands' else '0.4'"
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items(),
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom_gz@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/drone_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/lidar_3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        parameters=[{
            # This process does not need simulated time. More importantly,
            # keep only the newest clock sample so a delayed Gazebo message
            # cannot be delivered after a newer simulation timestamp.
            "qos_overrides./clock.publisher.history": "keep_last",
            "qos_overrides./clock.publisher.depth": 1,
            "qos_overrides./clock.publisher.reliability": "reliable",
            "qos_overrides./diff_drive_example.subscriber.reliability": "reliable",
        }],
        remappings=[
            ("/scan", "/scan"),
            ("/lidar_3d/points", "/points"),
            ("/imu", "/imu_raw"),
        ],
        output="screen",
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image"],
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "bot",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
        ],
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
    )

    pointcloud_transform = Node(
        package="bot_description",
        executable="pointcloud_transform",
        name="pointcloud_transform",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": "laser_link",
            "input_topic": "/points",
            "output_topic": "/velodyne_points",
            "scan_rate": 5.0,
            "num_scan_lines": 16,
            "vertical_fov_min": -15.0,
            "vertical_fov_max": 15.0,
            # Gazebo GPU LiDAR emits a full raycast snapshot per update, not a
            # sequential spinning scan.  Avoid synthetic per-point timing and
            # the resulting false LIO-SAM deskew.
            "assume_instantaneous_scan": True,
        }],
    )

    delayed_spawn = TimerAction(
        period=2.0,
        actions=[gz_spawn_entity],
    )

    delayed_processing = TimerAction(
        period=3.0,
        actions=[pointcloud_transform],
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        use_sim_time_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_ros2_bridge,
        ros_gz_image_bridge,
        delayed_spawn,
        delayed_processing,
    ])
