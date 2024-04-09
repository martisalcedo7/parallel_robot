import numpy as np
from time import sleep
from parallel_robot import ParallelRobotSimulation, Command


def control_client():

    MOTOR_DISTANCE = 0.08
    BASE_ARM_LENGTH = 0.05
    LINK_ARM_LENGTH = 0.08
    TCP_OFFSET = np.array([0.0, 0.00])

    robot_simulation = ParallelRobotSimulation(MOTOR_DISTANCE, BASE_ARM_LENGTH,
                                               LINK_ARM_LENGTH, TCP_OFFSET)

    r = 0.02
    command = Command(np.array([0.02619328 + r, 0.07476641]), False)
    robot_simulation.add_command(command)
    # sleep(2)
    points = 5
    for angle in range(points):
        x = r * np.cos(angle * 2 * np.pi / (points - 1)) + 0.02619328
        y = r * np.sin(angle * 2 * np.pi / (points - 1)) + 0.07476641
        command = Command(np.array([float(x), float(y)]), True)
        robot_simulation.add_command(command)
    command = Command(np.array([0.0, 0.06]), False)
    robot_simulation.add_command(command)

    robot_simulation.start()
    while True:
        print(robot_simulation.get_telemetry())
        sleep(0.1)
    robot_simulation.stop()
    # while not stop_event.is_set():
    #     telemetry = telemetry_sharer.get_telemetry()
    #     print(f"Current telemetry: {telemetry}")
    #     coordinates = input("Give command [cartesian_x,cartesian_y,drawing]: ")
    #     if coordinates == "exit":
    #         stop_event.set()
    #         break
    #     x, y, draw = coordinates.replace(" ", "").split(",")
    #     draw = True if draw == "True" else False
    #     command = Command(np.array([float(x), float(y)]), bool(draw))
    #     command_sharer.add_command(command)


if __name__ == "__main__":
    control_client()
