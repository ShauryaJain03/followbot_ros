import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bot_description = get_package_share_directory("bot_description")
    actor_plugin_description = get_package_share_directory("gazebo_ros_actor_plugin")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        bot_description, "urdf", "bot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    model_paths = [
        str(Path(bot_description).parent.resolve()),
        os.path.join(bot_description, "models"),
        os.path.join(actor_plugin_description, "config", "models"),
        os.path.join(actor_plugin_description, "config", "skins"),
    ]

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        pathsep.join(model_paths)
        )

    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model")
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='screen',
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    actor_baylands_world = os.path.join(
        actor_plugin_description, "config", "worlds", "baylands.world"
    )
    default_world_directory = os.path.join(bot_description, "worlds")
    world_path = PythonExpression([
        "'", actor_baylands_world, "' if '", LaunchConfiguration("world_name"),
        "' == 'baylands' else '", default_world_directory, "/' + '",
        LaunchConfiguration("world_name"), "' + '.world'"
    ])
    world_pose_bridge = PythonExpression([
        "'/world/' + '", LaunchConfiguration("world_name"),
        "' + '/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'"
    ])
    spawn_x = PythonExpression([
        "'-73.090393' if '", LaunchConfiguration("world_name"), "' == 'baylands' else '0.0'"
    ])
    spawn_y = PythonExpression([
        "'-112.385361' if '", LaunchConfiguration("world_name"), "' == 'baylands' else '0.0'"
    ])
    spawn_z = PythonExpression([
        "'5.0' if '", LaunchConfiguration("world_name"), "' == 'baylands' else '0.05'"
    ])

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
        arguments=[
            "-topic", "robot_description",
            "-name", "bot",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
        ],
    )


    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",            
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",            
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",            
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",            
            "/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",            
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",            
            "/drone_image@sensor_msgs/msg/Image@gz.msgs.Image",            
            "/thermal_camera_8bit/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/cmd_path@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V",
            world_pose_bridge,
            '/scan' + '@sensor_msgs/msg/LaserScan' + '[' + 'ignition.msgs.LaserScan',
            '/lidar_3d/points'  + '@sensor_msgs/msg/PointCloud2'   + '[' + 'ignition.msgs.PointCloudPacked',
        ],
        parameters=[
            {'qos_overrides./diff_drive_example.subscriber.reliability': 'reliable','use_sim_time': True}
        ],
        remappings= [
                    ('/scan',     '/scan'   ),
                    ('/lidar_3d/points', '/points'),
                    ('/imu','/imu_raw')
                    ],
        output="screen"
    )



    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image"],
        parameters=[{'use_sim_time': True}] 
    )




    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        world_name_arg,
        ros_gz_image_bridge,
        gz_ros2_bridge,
        joint_state_pub,

    ])
