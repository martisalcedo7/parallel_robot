import csv
import numpy as np
from time import sleep
from scipy.optimize import fsolve

from parallel_robot import ParallelRobotSimulation, Command, RobotConfiguration


def printing_area_function(h, robot_configuration: RobotConfiguration, ratio):
    hypotenuse = (robot_configuration.base_arm_length +
                  robot_configuration.link_arm_length)
    cathetus = ((h - robot_configuration.motor_distance) / 2.0 +
                robot_configuration.motor_distance)

    return (hypotenuse**2.0 - cathetus**
            2.0)**0.5 - np.abs(robot_configuration.base_arm_length -
                               robot_configuration.link_arm_length) - h * ratio


def load_points_from_csv(csv_file_path):
    points_array = []
    drawing_array = []
    with open(csv_file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            # Convert x and y to float, and the boolean according to its truth value
            x = float(row[0])
            y = float(row[1])
            draw = True if row[2] == 'true' else False
            points_array.append([x, y])
            drawing_array.append(draw)
    # Convert to a structured NumPy array for easier manipulation
    points_array = np.array(points_array, dtype=np.float64)
    return points_array, drawing_array


def process_points(points_array, robot_configuration):
    # Extract x, y coordinates, and drawing boolean
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
               args=(robot_configuration, ratio))

    h = h[0]
    v = h * ratio
    print("H painting area:", h)
    print("V painting area:", v)

    # Calculate origin of painting area
    y_prima = np.abs(robot_configuration.base_arm_length -
                     robot_configuration.link_arm_length)
    x_prima = (robot_configuration.motor_distance - h) * 0.5

    # Apply transformation equations to each point
    x_new = h * ((x_coords - min_x) / (max_x - min_x)) + x_prima
    y_new = v * ((y_coords - min_y) / (max_y - min_y)) + y_prima

    # Combine the transformed coordinates with the draw boolean
    transformed_points = np.column_stack((x_new, y_new))

    return transformed_points


def main():
    MOTOR_DISTANCE = 0.08
    BASE_ARM_LENGTH = 0.08
    LINK_ARM_LENGTH = 0.12
    TCP_OFFSET = np.array([0.0, 0.0])
    robot_simulation = ParallelRobotSimulation(MOTOR_DISTANCE, BASE_ARM_LENGTH,
                                               LINK_ARM_LENGTH, TCP_OFFSET)
    robot_configuration = robot_simulation.get_robot_configuration()

    points_array, drawing_array = load_points_from_csv('drawing.csv')
    transformed_points_array = process_points(
        points_array=points_array, robot_configuration=robot_configuration)

    for point, draw in zip(transformed_points_array, drawing_array):
        robot_simulation.add_command(Command(point, draw))

    robot_simulation.start()
    robot_simulation.wait_until_no_commands()
    sleep(5.0)
    robot_simulation.stop()


if __name__ == "__main__":
    main()
