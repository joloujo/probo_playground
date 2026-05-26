from environment import Environment
from math import pi
import pytest
from robot import Robot
from sensors import LandmarkPinger, WheelEncoderDifferential
from utils import Position, Pose, Bounds, Landmark, BearingRange

class TestSensors:

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

    @pytest.fixture
    def landmark_pinger(self, robot: Robot):
        return LandmarkPinger(robot, range_noise=0, range_prop_noise=0, bearing_noise=0, max_range=3)
    
    @pytest.fixture
    def wheel_encoder(self, robot: Robot):
        return WheelEncoderDifferential(robot, lin_noise=0, ang_noise=0)

    def test_landmark_pinger_sample(self, robot: Robot, landmark_pinger: LandmarkPinger):
        # Make sure landmarks out of range arent sensed 
        assert landmark_pinger.sample() == [BearingRange(0, float('inf'), float('inf'))]

        # Move within range
        robot.env.robot_step(0, 2, 0)

        # Make sure landmarks within range are sensed
        assert landmark_pinger.sample() == [BearingRange(0, pi/2, 2)]

    def test_wheel_encoder_sample(self, robot: Robot, wheel_encoder: WheelEncoderDifferential):
        assert wheel_encoder.sample() == (0, 0)
        
        robot.robot_step_differential(0.5, 0)

        assert wheel_encoder.sample() == (0.5, 0)

        robot.robot_step_differential(0, pi/2)

        assert wheel_encoder.sample() == (0, pi/2)

        robot.robot_step_differential(1, pi/2)

        assert wheel_encoder.sample() == (1, pi/2)
