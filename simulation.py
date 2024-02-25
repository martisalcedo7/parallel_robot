from tools import TimeManager
from robot import Robot
import numpy as np
from trajectories import constant_velocity

SAMPLING_TIME = 0.01


def simulation(stop_event, telemetry_sharer, command_sharer):
    # Robot
    initial_joint_position = np.array([2.5, 0.6])
    parallel_robot = Robot(initial_joint_position)

    # Clock to control the frame rate
    time_manager = TimeManager(SAMPLING_TIME)

    counter = 0
    trajectory = []
    # Main game loop
    while not stop_event.is_set():

        telemetry = parallel_robot.get_telemetry()
        telemetry_sharer.update_telemetry(telemetry)

        command = command_sharer.get_command()
        if command is not None and counter == 0:
            trajectory = constant_velocity(telemetry.cartesian_position,
                                           command.cartesian_position, 0.1,
                                           SAMPLING_TIME)
            parallel_robot.set_drawing(command.drawing)

        if counter < len(trajectory):
            parallel_robot.set_cartesian_position(trajectory[counter])
            counter += 1
        else:
            counter = 0
            trajectory = []
            parallel_robot.set_drawing(False)

        # Cap the frame rate
        time_manager.adaptive_sleep()


if __name__ == "__main__":
    simulation()
