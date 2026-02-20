"""
A simulation environment for a mobile robot operating in two dimensions.

The Environment class models the world that the robots navigate in. The world is continuous and two-dimensional. The world possesses an outer border, internal obstacles, and identifiable landmarks. The world also manages the passage of time and the motion of robotic agents within the world over time.

Critically, the environment tracks the robot's state. In this case, the robot's state is a vector that includes three state variables: x position, y position, and heading.
"""

from math import pi
from utils import Position, Pose, Bounds, Landmark, BearingRange, State
import pandas as pd

class Environment:
    """
    A class that models the world simulation environment and the robot's state.

    Attributes:
        dimensions: the horizontal and vertical size of the world
        dt: the length of each timestep, in seconds
        obstacles: a list of obstacles
        landmarks: a list of landmarks
        robot_pose: the position and heading of the robot in the world
    """

    def __init__(
        self,
        dimensions: Bounds,
        dt: float,
        obstacles: list[Bounds],
        landmarks: list[Landmark],
        robot_starting_pose: Pose,
    ) -> None:
        """
        Initialize an instance of the Environment class.

        Args:
            dimensions: the horizontal and vertical size of the world
            dt: the length of each timestep, in seconds
            obstacles: a list of obstacles
            landmarks: a list of landmarks
            robot_starting_pose: the initial position and heading of the robot
        """
        self.DIMENSIONS = dimensions

        self.DT = dt

        self.time: float = 0

        self.OBSTACLES = obstacles
        self.LANDMARKS = landmarks

        self.robot_starting_pose = robot_starting_pose
        self.robot_pose = robot_starting_pose.copy()

    def robot_step(self, dx: float, dy: float, dtheta: float) -> None:
        """
        Update the robot's position and heading in the world. The robot should not be able to pass through obstacles or outside of the world bounds.

        Args:
            dx: change in x position
            dy: change in y position
            dtheta: change in heading

        Returns:
            Nothing, but update the robot_pose property at the end
        """

        # Get the x and y motion that the robot can validly make
        valid_dx, valid_dy = self.validate_xy_motion(dx, dy)

        # Update the robot's position and orientation
        self.robot_pose.pos += Position(valid_dx, valid_dy)
        self.robot_pose.theta += dtheta

        # Update the simulator timestep
        self.time += self.DT


    def validate_xy_motion(self, dx: float, dy: float) -> tuple[float, float]:
        """
        Given attempted x and y motion by the robot, determine what motion is physically possible (i.e. doesn't go through any obstacles or barriers). Return the actual motion that will be executed.

        Args:
            dx: attempted change in x position
            dy: attempted change in y position

        Returns:
            dx: change in x position that should be executed
            dy: change in y position that should be executed
        """
        
        # TODO: Implement this with ray tracing for more accurate simulation

        # # Loop through all things that stop motion
        # for bound in [self.DIMENSIONS] + self.OBSTACLES:

        return (dx, dy) \
            if self.is_valid_position(self.robot_pose.pos + Position(dx, dy)) \
            else (0, 0)

    def is_valid_position(self, position: Position) -> bool:
        """
        Check if a given robot position is valid; i.e. not out-of-bounds or within an obstacle. Return a boolean representing whether or not this condition is true.

        Args:
            position: the robot position

        Returns:
            true if the position is valid and false otherwise
        """
        
        # Check if the position is out of the environment bounds
        if not self.DIMENSIONS.within_bounds(position):
            return False

        # For each obstacle, check if the position is in it
        for obstacle in self.OBSTACLES:
            if obstacle.within_bounds(position):
                return False

        # If the position is not outside the environment bounds and not in any obstacle bounds, then it is a valid position
        return True

    def get_robot_pose(self):
        """
        Return the true robot pose.
        """
        return self.robot_pose

    def get_proximity_to_landmarks(self) -> list[BearingRange]:
        """
        Return a list of the robot's true range and bearing to all landmarks.
        """
        return [
            BearingRange(
                landmark_id=landmark.id,
                range=(landmark.pos - self.robot_pose.pos).magnitude,
                bearing=(((landmark.pos - self.robot_pose.pos).angle - self.robot_pose.theta) + pi) % (2 * pi) - pi,
            ) for landmark in self.LANDMARKS
        ] 

    def take_state_snapshot(self):
        """
        Return true state information about this timestep, including time, robot position, and the robot's bearing/range to landmarks, in a table format.
        """
        state: State = {
            'time': [self.time],
            'robot_pose': [self.robot_pose.copy()],
            'landmark_br': [self.get_proximity_to_landmarks()],
        }
        return pd.DataFrame(state)

    def get_environment_info(self):
        """
        Return static information about the environment, including dimensions, timestep size, locations and dimensions of obstacles, and locations of landmarks.
        """
        return {
            'dimensions': self.DIMENSIONS,
            'dt': self.DT,
            'obstacles': self.OBSTACLES,
            'landmarks': self.LANDMARKS,
        }
