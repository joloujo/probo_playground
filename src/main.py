"""
Main file for running the simulator.
"""

import csv
from environment import Environment
from pathlib import Path
from robot import Robot
import pandas as pd
from utils import Position, Pose, Landmark, Bounds

if __name__ == "__main__":
    # set up the environment
    dimensions = Bounds(0, 10, 0, 10)
    dt = 0.1
    obstacles = [
        Bounds(0, 4, 2, 4),
        Bounds(2, 4, 6, 8),
        Bounds(6, 8, 0, 4),
    ]
    landmarks = [
        Landmark(Position(1, 9), 0),
        Landmark(Position(3, 3), 1),
        Landmark(Position(5, 7), 2),
        Landmark(Position(9, 1), 3),
        Landmark(Position(9, 9), 4),
    ]
    initial_robot_pose = Pose(Position(1, 1), 0)

    env = Environment(
        dimensions,
        dt,
        obstacles,
        landmarks,
        initial_robot_pose,
    )

    # set up the robot
    robot = Robot(env)

    # set up timekeeping
    total_seconds = 10
    total_timesteps = total_seconds / env.DT

    # set up logging
    ground_truth_history = pd.DataFrame()
    sensor_data_history = pd.DataFrame()

    # set up input filepath and output filepaths
    input_commands_filepath = Path('./input/cmd_vel.csv')
    output_ground_truth_filepath = Path('./output/ground_truth.csv')
    output_sensor_data_filepath = Path('./output/sensor_data.csv')

    # open up the instructions, pop the first
    with open(input_commands_filepath, "r") as input_commands_file:
        
        # Set up the csv reader
        commands = csv.reader(input_commands_file)
        # Skip the headers
        next(commands)
        # Get the first command
        next_command = next(commands)

        lin_vel: float = 0
        ang_vel: float = 0

        # iterate through each timestep
        for step in range(int(total_timesteps) + 1):
            # Take a ground truth snapshot and add it to the history
            print(env.take_state_snapshot())
            ground_truth_history = pd.concat([ground_truth_history, env.take_state_snapshot()], axis=0, ignore_index=True)

            # Take sensor measurements and add it to the history
            sensor_data_history = pd.concat([sensor_data_history, robot.take_sensor_measurements()], axis=0, ignore_index=True)

            # Retrieve the next motor command from the input file
            if next_command is not None and float(next_command[0]) <= env.time:
                lin_vel = float(next_command[1])
                ang_vel = float(next_command[2])

                next_command = next(commands, None)

            # Execute the motor command
            env.robot_step(*robot.robot_step_differential(lin_vel, ang_vel))

    # at the end, write the histories into output files
    with open(output_ground_truth_filepath, "w") as gt_data:
        ground_truth_history.to_csv(gt_data)

    with open(output_sensor_data_filepath, "w") as sensor_data:
        sensor_data_history.to_csv(sensor_data)
