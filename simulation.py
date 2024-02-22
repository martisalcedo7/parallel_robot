from tools import TimeManager
from robot import Robot
import numpy as np
from trajectories import constant_velocity

SAMPLING_TIME = 0.01

def simulation(telemetry_sharer, command_sharer):
    # Robot
    initial_joint_position = np.array([2.5, 0.6])
    parallel_robot = Robot(initial_joint_position)

    # Clock to control the frame rate
    time_manager = TimeManager(SAMPLING_TIME)

    counter = 0
    trajectory = []
    # Main game loop
    while True:

        telemetry_sharer.update_telemetry(parallel_robot.get_cartesian_position())

        command = command_sharer.get_command()
        if command is not None:
            trajectory = constant_velocity(parallel_robot.get_cartesian_position(), command, 0.1, SAMPLING_TIME)

        if counter < len(trajectory):
            parallel_robot.set_cartesian_position(trajectory[counter])
            counter += 1
        else:
            counter = 0
            trajectory = []

        # Cap the frame rate
        time_manager.adaptive_sleep()


if __name__ == "__main__":
    simulation()
