import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bot_description = get_package_share_directory("bot_description")
    bot_controller = get_package_share_directory("bot_controller")
    default_rviz_config = os.path.join(bot_description, "rviz", "display.rviz")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(bot_description, "urdf", "bot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="empty",
        description="World file name (without .world extension)"
    )
    use_ground_truth_odom_arg = DeclareLaunchArgument(
        "use_ground_truth_odom",
        default_value="true",
        description=(
            "Use the controller configuration for an external odometry "
            "source; it disables the controller odom TF publisher."
        ),
    )
    publish_ground_truth_odom_arg = DeclareLaunchArgument(
        "publish_ground_truth_odom",
        default_value="true",
        description=(
            "Publish Gazebo ground truth as /odom and odom -> base_link. "
            "Set false when LIO-SAM owns odom -> base_link."
        ),
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz for the simulation bringup"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use the Gazebo /clock source for every simulation node.",
    )

    model = LaunchConfiguration("model")
    world_name = LaunchConfiguration("world_name")
    is_baylands_world = PythonExpression([
        "'", world_name, "' == 'baylands'"
    ])
    use_ground_truth_odom = LaunchConfiguration("use_ground_truth_odom")
    publish_ground_truth_odom = LaunchConfiguration("publish_ground_truth_odom")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    controller_params_file = PythonExpression([
        "'", os.path.join(bot_controller, "config", "bot_controller_ground_truth.yaml"),
        "' if '", use_ground_truth_odom, "' == 'true' else '",
        os.path.join(bot_controller, "config", "bot_controller.yaml"), "'"
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bot_description,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'model': model,
            'world_name': world_name,
            'controller_params_file': controller_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bot_controller'),
                'launch',
                'controller.launch.py'
            )
        )
    )

    world_pose_topic = PythonExpression([
        "'/world/' + '", world_name, "' + '/pose/info'"
    ])

    ground_truth_odom = Node(
        package='bot_terrain_follower',
        executable='robot_ground_truth_publisher',
        name='robot_ground_truth_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'world_pose_topic': world_pose_topic,
            'robot_entity_name': 'bot',
            'pose_topic': '/robot_pose_gt',
            'map_frame_id': 'map',
            'odom_topic': '/odom',
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'publish_tf': True,
        }],
        condition=IfCondition(publish_ground_truth_odom),
        output='screen',
    )

    actor_path_publisher = Node(
        package='gazebo_ros_actor_plugin',
        executable='path_publisher.py',
        name='actor_path_publisher',
        condition=IfCondition(is_baylands_world),
        parameters=[{
            'cmd_path_delay': 5.0,
        }],
        output='screen',
    )

    rviz_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d',
                    default_rviz_config
                ],
                parameters=[{
                    'use_sim_time': use_sim_time,
                }],
                output='screen',
            )
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        use_ground_truth_odom_arg,
        publish_ground_truth_odom_arg,
        use_rviz_arg,
        use_sim_time_arg,
        gazebo,
        controller,
        ground_truth_odom,
        actor_path_publisher,
        rviz_node
    ])
