from dataclasses import dataclass
from environment import Environment
from math import sin, cos
from matplotlib.axes import Axes
from matplotlib.patches import Circle
import matplotlib.pyplot as plt
from pathlib import Path
import pandas as pd
from robot import Robot
from utils import Bounds, Pose, Position

@dataclass
class PlotDataOptions:
    pinger_range: float | None

class Visualizer:
    """
    Visualizer for Kalman Filter trajectory estimation results.
    """

    def __init__(self, env: Environment, robot: Robot | None = None) -> None:
        self.env = env
        self.robot = robot

    def plot_data(self, 
        ground_truth: pd.DataFrame | None = None, 
        sensor_data: pd.DataFrame | None = None, 
        options: PlotDataOptions = PlotDataOptions(None)
    ):
        fig, ax = plt.subplots()

        # Plot the environment

        # Plot each obstacle
        for obstacle in self.env.OBSTACLES:
            self.plot_bounds(obstacle, ax, color='black')

        # Plot the bounds 
        self.plot_bounds(self.env.DIMENSIONS, ax, color='black')

        # Plot each landmark
        for landmark in self.env.LANDMARKS:
            ax.plot(landmark.pos.x, landmark.pos.y, '*', color='black', ms='10')
            if options.pinger_range is not None:
                ax.add_patch(Circle((landmark.pos.x, landmark.pos.y), options.pinger_range, fill=False, color='black', ls='--'))

        # Plot the ground truth data
        if ground_truth is not None:
            self.plot_poses(ground_truth['robot_pose'].tolist(), ax, alpha=0.5, color='blue') # type: ignore

        # Plot the ground truth data
        if sensor_data is not None:

            if self.robot is None:
                print("Can't print sensor data without a robot")
            else:
                sensor_poses: list[Pose] = [self.env.robot_starting_pose]
                # (liner_vel, angular_vel)
                encoder_values: pd.Series[tuple[float, float]] = sensor_data['wheel_encoder'] # type: ignore
                
                for reading in encoder_values:
                    last_pose = sensor_poses[-1]
                    dx, dy, dtheta = self.robot.robot_step_differential_from_arbitrary_pose(last_pose, reading[0], reading[1])
                    sensor_poses.append(Pose(last_pose.pos + Position(dx, dy), last_pose.theta + dtheta))

                self.plot_poses(sensor_poses, ax, alpha=0.5, color='red')

        ax.set_aspect('equal')
        plt.xlim(self.env.DIMENSIONS.x_min - 1, self.env.DIMENSIONS.x_max + 1)
        plt.ylim(self.env.DIMENSIONS.y_min - 1, self.env.DIMENSIONS.y_max + 1)
        plt.show()
    
    def plot_bounds(self, bounds: Bounds, ax: Axes, **kwargs):
        # Plot the bounds 
        bounds_x = [
            bounds.x_min,
            bounds.x_max,
            bounds.x_max,
            bounds.x_min,
            bounds.x_min,
        ]
        bounds_y = [
            bounds.y_min,
            bounds.y_min,
            bounds.y_max,
            bounds.y_max,
            bounds.y_min,
        ]
        ax.plot(bounds_x, bounds_y, **kwargs)

    def plot_poses(self, poses: list[Pose], ax: Axes, **kwargs):
        x: list[float]
        y: list[float]
        theta: list[float]
        x, y, theta = map(list, zip(*[(pose.pos.x, pose.pos.y, pose.theta) for pose in poses]))

        u: list[float]
        v: list[float]
        u, v = map(list, zip(*[(cos(t), sin(t)) for t in theta]))

        ax.quiver(x, y, u, v, **kwargs)