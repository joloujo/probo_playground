"""
A simulated robotic agent with teleoperation and sensing capabilities.

The Robot class models the robotic agent that explores the world. The robot is remote-controlled by angular and linear velocity commands read from an external file. The robot can execute motor commands to move, and can sense both externally (GPS, landmarks, obstacles) and internally (odometry, IMU).
"""

from environment import Environment
from math import sin, cos
from random import gauss
from sensors import SensorInterface


class Robot:
    """
    A class that models a simulated robotic agent.

    Attributes:
        env: the environment this robot is operating in
        sensors: list of all robot sensors
    """

    def __init__(self, env: Environment, lin_vel_noise: float = 0, ang_vel_noise: float = 0):
        """
        Initialize an instance of the Robot class.

        Args:
            env: the environment this robot is operating in
            lin_vel_noise: the proportion of the command linear velocity to use for the standard deviation of the gaussian noise applied to the command linear velocity 
            ang_vel_noise: the proportion of the command angular velocity to use for the standard deviation of the gaussian noise applied to the command angular velocity 
        """
        self.env = env

        self.lin_vel_noise = lin_vel_noise
        self.ang_vel_noise = ang_vel_noise

        self.sensors = []

    def robot_step_differential(self, lin_vel: float, ang_vel: float) -> tuple[float, float, float]:
        """
        Differential-drive mode. Given forward linear and angular velocities, determine the robot's change in x, y, and heading and apply those changes in the environment.

        Args:
            lin_vel: input linear velocity command
            ang_vel: input angular velocity command

        Returns:
            dx: change in x position
            dy: change in y position
            d-theta: change in heading
        """

        noisy_lin_vel = gauss(lin_vel, lin_vel * self.lin_vel_noise)
        noisy_ang_vel = gauss(ang_vel, ang_vel * self.ang_vel_noise)

        if noisy_ang_vel == 0: 
            return (
                noisy_lin_vel * cos(self.env.robot_pose.theta),
                noisy_lin_vel * sin(self.env.robot_pose.theta),
                0
            )
        elif noisy_lin_vel == 0: # Not strictly necessary, but should improve performance slightly
            return (
                0,
                0,
                noisy_ang_vel * self.env.DT
            )
        else:
            return (
                (noisy_lin_vel / noisy_ang_vel) * (sin(self.env.robot_pose.theta + noisy_ang_vel * self.env.DT) - sin(self.env.robot_pose.theta)),
                -(noisy_lin_vel / noisy_ang_vel) * (cos(self.env.robot_pose.theta + noisy_ang_vel * self.env.DT) - cos(self.env.robot_pose.theta)),
                noisy_ang_vel * self.env.DT
            )

    def robot_step_translational(self, x_vel: float, y_vel: float, ang_vel: float):
        """
        Swerve-drive mode. Given x, y, and angular velocities, determine the robot's change in x, y, and heading and apply those changes in the environment.

        Args:
            x_vel: input x velocity command
            y_vel: input y velocity command
            ang_vel: input angular velocity command

        Returns:
            dx: change in x position
            dy: change in y position
            d-theta: change in heading
        """
        # TODO: fill in the function
        pass

    def take_sensor_measurements(self):
        """
        Return noisy sensor readings of the environment at this timestep, including data from all sensors, in a table format.
        """
        # TODO: fill in the function
        pass
