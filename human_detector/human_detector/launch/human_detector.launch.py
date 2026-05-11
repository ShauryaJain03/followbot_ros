from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events.matchers import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument("use_sim", default_value="false", description="Use sim time.")
    pose_model_arg = DeclareLaunchArgument(
        "pose_landmarker_model_path",
        default_value="",
        description="Absolute path to the MediaPipe pose landmarker .task model.",
    )
    horizontal_fov_arg = DeclareLaunchArgument(
        "horizontal_fov",
        default_value="0.0",
        description="Horizontal camera FOV in radians. If > 0, use it instead of CameraInfo intrinsics.",
    )
    camera_frame_arg = DeclareLaunchArgument(
        "camera_frame_id",
        default_value="camera_link",
        description="Frame ID used as the parent for the detected human transform.",
    )
    rgb_topic_arg = DeclareLaunchArgument(
        "rgb_image_topic",
        default_value="/camera/color/image_raw",
        description="RGB image topic used for human detection.",
    )
    depth_topic_arg = DeclareLaunchArgument(
        "depth_image_topic",
        default_value="/camera/depth/image_rect_raw",
        description="Depth image topic used for human detection.",
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/depth/camera_info",
        description="CameraInfo topic corresponding to the aligned RGB/depth streams.",
    )
    human_detector = LifecycleNode(
        package="human_detector",
        executable="human_detector",
        name="human_detector",
        output="screen",
        namespace="",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim")},
            {"pose_landmarker_model_path": LaunchConfiguration("pose_landmarker_model_path")},
            {"horizontal_fov": LaunchConfiguration("horizontal_fov")},
            {"camera_frame_id": LaunchConfiguration("camera_frame_id")},
            {"rgb_image_topic": LaunchConfiguration("rgb_image_topic")},
            {"depth_image_topic": LaunchConfiguration("depth_image_topic")},
            {"camera_info_topic": LaunchConfiguration("camera_info_topic")},
        ],
    )

    move_human_detector_to_configure_state_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(human_detector),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    return LaunchDescription(
        [
            use_sim_arg,
            pose_model_arg,
            horizontal_fov_arg,
            camera_frame_arg,
            rgb_topic_arg,
            depth_topic_arg,
            camera_info_topic_arg,
            human_detector,
            move_human_detector_to_configure_state_event,
        ]
    )
