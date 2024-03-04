from .tools import TimeManager, Telemetry
from .robot import ParallelRobot
from .trajectories import constant_velocity

SAMPLING_TIME = 0.001


def simulation(stop_event, telemetry_sharer, command_sharer,
               robot_configuration):
    try:
        # Robot
        parallel_robot = ParallelRobot(*robot_configuration)

        # Clock to control the frame rate
        time_manager = TimeManager(SAMPLING_TIME)

        counter = 0
        trajectory = []
        # Main game loop
        while not stop_event.is_set():

            state = parallel_robot.get_state()
            telemetry = Telemetry(*state)
            telemetry_sharer.update_telemetry(telemetry)

            if counter == 0:
                command = command_sharer.get_command()
                if command is not None:
                    trajectory = constant_velocity(state[1],
                                                   command.cartesian_position,
                                                   0.1, SAMPLING_TIME)
                    parallel_robot.set_drawing(command.drawing)

            if counter < len(trajectory):
                if parallel_robot.in_workspace(trajectory[counter]):
                    parallel_robot.set_cartesian_position(trajectory[counter])
                counter += 1
            else:
                counter = 0
                trajectory = []
                # parallel_robot.set_drawing(False)

            # Control frequency
            time_manager.adaptive_sleep()

    except Exception as e:
        print(f"Exception {e}. Stopping simulation.")
        stop_event.set()


if __name__ == "__main__":
    simulation()
