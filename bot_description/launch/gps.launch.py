import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration,PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bot_description = get_package_share_directory("bot_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        bot_description, "urdf", "bot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    model_path = str(Path(bot_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("bot_description"), 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
        )

    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model")
        ]),
        value_type=str
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bot_description, "rviz", "gps.rviz")],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='screen',
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    world_path = PathJoinSubstitution([
            bot_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    joint_state_pub = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time' : True}]
        )
        

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "bot"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            '/scan' + '@sensor_msgs/msg/LaserScan' + '[' + 'ignition.msgs.LaserScan',
            '/scan/points/points'  + '@sensor_msgs/msg/PointCloud2'   + '[' + 'ignition.msgs.PointCloudPacked',],
        parameters= [{'qos_overrides./diff_drive_example.subscriber.reliability': 'reliable'}],
        remappings= [
                    ('/scan',     '/scan'   ),
                    ('/scan/points/points', '/points'),
                    ],
        output="screen"
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )




    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        world_name_arg,
        rviz_node,
        ros_gz_image_bridge,
        gz_ros2_bridge,
        joint_state_pub
    ])