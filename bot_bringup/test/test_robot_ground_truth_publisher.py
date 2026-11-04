import math

from bot_bringup.robot_ground_truth_publisher import RobotGroundTruthPublisher


def test_velocity_is_expressed_in_base_frame():
    linear_x, linear_y = RobotGroundTruthPublisher._velocity_in_base_frame(
        0.0, 1.0, math.pi / 2.0)

    assert math.isclose(linear_x, 1.0)
    assert math.isclose(linear_y, 0.0, abs_tol=1e-12)
