"""
A simulated robotic agent with teleoperation and sensing capabilities.

The Robot class models the robotic agent that explores the world. The robot is remote-controlled by angular and linear velocity commands read from an external file. The robot can execute motor commands to move, and can sense both externally (GPS, landmarks, obstacles) and internally (odometry, IMU).
"""

from environment import Environment
from math import sin, cos
import pandas as pd
from random import gauss
from sensors import SensorInterface, WheelEncoderDifferential, LandmarkPinger
from utils import Pose

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

        self.cmd_lin_vel: float = 0
        self.cmd_ang_vel: float = 0

        self.actual_lin_vel: float = 0
        self.actual_ang_vel: float = 0

        self.sensors: list[SensorInterface] = [
            WheelEncoderDifferential(self),
            LandmarkPinger(self),
        ]

    def robot_step_differential_from_arbitrary_pose(self, start: Pose, lin_vel: float, ang_vel: float):
        """
        Given a starting pose and forward linear and angular velocities, determine the robot's change in x, y, and heading.

        Args:
            start: the pose of the robot before the movement step
            lin_vel: input linear velocity command
            ang_vel: input angular velocity command

        Returns:
            dx: change in x position
            dy: change in y position
            d-theta: change in heading
        """

        self.cmd_lin_vel = lin_vel
        self.cmd_ang_vel = ang_vel

        noisy_lin_vel = gauss(lin_vel, lin_vel * self.lin_vel_noise)
        noisy_ang_vel = gauss(ang_vel, ang_vel * self.ang_vel_noise)

        self.actual_lin_vel = noisy_lin_vel
        self.actual_ang_vel = noisy_ang_vel

        if noisy_ang_vel == 0: 
            return (
                noisy_lin_vel * cos(start.theta) * self.env.DT,
                noisy_lin_vel * sin(start.theta) * self.env.DT,
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
                (noisy_lin_vel / noisy_ang_vel) * (sin(start.theta + noisy_ang_vel * self.env.DT) - sin(start.theta)),
                -(noisy_lin_vel / noisy_ang_vel) * (cos(start.theta + noisy_ang_vel * self.env.DT) - cos(start.theta)),
                noisy_ang_vel * self.env.DT
            )        

    def robot_step_differential(self, lin_vel: float, ang_vel: float) -> tuple[float, float, float]:
        """
        Differential-drive mode. Given forward linear and angular velocities, determine the robot's change in x, y, and heading.

        Args:
            lin_vel: input linear velocity command
            ang_vel: input angular velocity command

        Returns:
            dx: change in x position
            dy: change in y position
            d-theta: change in heading
        """

        return self.robot_step_differential_from_arbitrary_pose(self.env.robot_pose, lin_vel, ang_vel)        

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
        return (
            x_vel * self.env.DT,
            y_vel * self.env.DT,
            ang_vel * self.env.DT
        )

    def take_sensor_measurements(self):
        """
        Return noisy sensor readings of the environment at this timestep, including data from all sensors, in a table format.
        """
        
        samples = {
            sensor.name: [sensor.sample()]
            for sensor in self.sensors
            if self.env.time >= sensor.last_meas_t + sensor.interval 
        }

        return pd.DataFrame(samples)


