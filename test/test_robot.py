from environment import Environment
from math import pi
import pytest
from robot import Robot
from utils import Position, Pose, Bounds, Landmark, BearingRange

class TestRobot:

    @pytest.fixture
    def robot(self):
        environment = Environment(
            dimensions=Bounds(0, 10, 0, 10),
            dt=1,
            obstacles=[
                Bounds(2, 4, 0, 2),
            ],
            landmarks=[
                Landmark(Position(1, 5), 0)
            ],
            robot_starting_pose=Pose(Position(1, 1), 0)
        )

        return Robot(
            env=environment
        )

    def test_robot_step_differential(self, robot: Robot):
        assert robot.robot_step_differential(0, pi/2) == (0, 0, pi/2)
        assert robot.robot_step_differential(1, 0) == (1, 0, 0)
        assert robot.robot_step_differential(1, pi/2) == pytest.approx((2/pi, 2/pi, pi/2))

        # Turn the robot to face north
        robot.env.robot_step(0, 0, pi/2)

        assert robot.robot_step_differential(1, -pi/2) == pytest.approx((2/pi, 2/pi, -pi/2))