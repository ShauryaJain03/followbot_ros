import math

from bot_bringup.person_ground_truth_publisher import PersonGroundTruthPublisher


def test_world_pose_is_expressed_at_robot_odom_origin():
    x, y, yaw = PersonGroundTruthPublisher._to_odom(
        -218.5, -68.0, math.pi / 2.0, -222.0, -68.0, 0.0)

    assert math.isclose(x, 3.5)
    assert math.isclose(y, 0.0)
    assert math.isclose(yaw, math.pi / 2.0)
