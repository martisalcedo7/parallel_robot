import csv
import numpy as np
from time import sleep
from scipy.optimize import fsolve

from parallel_robot import ParallelRobotSimulation, Command, Telemetry, RobotConfiguration


def printing_area_function(h, robot_configuration: RobotConfiguration, ratio):
    hypotenuse = (robot_configuration.base_arm_length +
                  robot_configuration.link_arm_length)
    cathetus = ((h - robot_configuration.motor_distance) / 2.0 +
                robot_configuration.motor_distance)

    return (hypotenuse**2.0 - cathetus**
            2.0)**0.5 - np.abs(robot_configuration.base_arm_length -
                               robot_configuration.link_arm_length) - h * ratio


def load_points_from_csv(csv_file_path):
    points_array = None
    with open(csv_file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        points = list(csv_reader)
        points_array = np.array(points, dtype=np.float64)
    return points_array


def process_points(points_array, robot_configuration):

    # Extract x and y coordinates
    x_coords = points_array[:, 0]
    y_coords = points_array[:, 1]

    # Find max and min values of x and y
    max_x = np.max(x_coords)
    min_x = np.min(x_coords)
    max_y = np.max(y_coords)
    min_y = np.min(y_coords)

    print("Max x:", max_x)
    print("Min x:", min_x)
    print("Max y:", max_y)
    print("Min y:", min_y)

    ratio = (max_y - min_y) / (max_x - min_x)
    print("Ratio=V/H:", ratio)

    h = fsolve(printing_area_function,
               robot_configuration.link_arm_length,
               args=(
                   robot_configuration,
                   ratio,
               ))

    h = h[0]
    v = h * ratio
    print("H painting area:", h)
    print("V painting area:", v)

    # Calculate origin of painting area
    y_prima = np.abs(robot_configuration.base_arm_length -
                     robot_configuration.link_arm_length)
    x_prima = (robot_configuration.motor_distance - h) * 0.5

    # Process points
    # Apply transformation equations to each point
    x_new = h * ((x_coords - min_x) / (max_x - min_x)) + x_prima
    y_new = v * ((y_coords - min_y) / (max_y - min_y)) + y_prima

    # Create a new array with transformed coordinates
    transformed_points = np.column_stack((x_new, y_new))

    return transformed_points


def main():
    MOTOR_DISTANCE = 0.08
    BASE_ARM_LENGTH = 0.05
    LINK_ARM_LENGTH = 0.08
    TCP_OFFSET = np.array([0.0, 0.0])
    robot_simulation = ParallelRobotSimulation(MOTOR_DISTANCE, BASE_ARM_LENGTH,
                                               LINK_ARM_LENGTH, TCP_OFFSET)
    robot_configuration = robot_simulation.get_robot_configuration()

    points_array = load_points_from_csv('drawing.csv')
    transformed_points_array = process_points(
        points_array=points_array, robot_configuration=robot_configuration)

    draw = False
    for point in transformed_points_array:
        robot_simulation.add_command(Command(point, draw))
        draw = True

    robot_simulation.start()
    sleep(30)
    robot_simulation.stop()


if __name__ == "__main__":
    main()
