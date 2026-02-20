"""
A class to test the Environment class
"""
from environment import Environment
from math import pi
import pandas as pd
import pytest
from utils import Position, Pose, Bounds, Landmark, BearingRange

class TestEnvironment:

    @pytest.fixture
    def environment(self) -> Environment:
        return Environment(
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

    def test_initialization(self, environment: Environment):
        assert environment.DIMENSIONS == Bounds(0, 10, 0, 10)
        assert environment.DT == 1
        assert environment.OBSTACLES == [
            Bounds(2, 4, 0, 2),
        ]
        assert environment.LANDMARKS == [
            Landmark(Position(1, 5), 0)
        ]
        assert environment.robot_pose == Pose(Position(1, 1), 0)
    
    def test_is_valid_position(self, environment: Environment):
        # Make sure that somewhere in the bounds of the environment and outside of any obstacles is valid
        assert environment.is_valid_position(Position(1, 1))

        # Make sure that outside the environment is invalid
        assert not environment.is_valid_position(Position(-1, -1))
        assert not environment.is_valid_position(Position(11, 5))
        assert not environment.is_valid_position(Position(5, 11))

        # Make sure that inside obstacles are invalid
        assert not environment.is_valid_position(Position(3, 1))

        # TODO: Decide if robots can be exactly on walls (if bounds are inclusive or excluside)

    def test_robot_moves(self, environment: Environment):
        # Move the robot
        environment.robot_step(0, 1, 1)

        assert environment.robot_pose == Pose(Position(1, 2), 1)

    def test_robot_cannot_move_out_of_bounds(self, environment: Environment):
        # Try to move the robot out of the bounds of the environment
        environment.robot_step(-2, 0, 0)

        assert environment.is_valid_position(environment.robot_pose.pos)

        # TODO: Check if the robot still moves as much as it can, once that's implemented

    def test_robot_cannot_move_into_obstacles(self, environment: Environment):
        # Try to move the robot into the first obstacle
        environment.robot_step(2, 0, 0)

        assert environment.is_valid_position(environment.robot_pose.pos)

        # TODO: Check if the robot still moves as much as it can, once that's implemented

    @pytest.mark.xfail(reason="This functionality hasn't been implemented yet")
    def test_robot_cannot_move_through_obstacles(self, environment: Environment):
        # Try to move the robot fully through the first obstacle
        environment.robot_step(4, 0, 0)

        assert environment.is_valid_position(environment.robot_pose.pos)

    def test_get_proximity_to_landmarks(self, environment: Environment):
        assert environment.get_proximity_to_landmarks() == [BearingRange(0, pi/2, 4)]

        # Move the robot
        environment.robot_step(0, 1, pi/2)

        assert environment.get_proximity_to_landmarks() == [BearingRange(0, 0, 3)]
