import sys
import pygame
import numpy as np
from copy import copy

from tools import TimeManager
from robot import Robot
from trajectories import constant_velocity

# Screen dimensions
screen_width = 800
screen_height = 600

pixel_per_metre = 1800


def scale(value):
    return value * pixel_per_metre


def inverse_scale(value):
    return value / pixel_per_metre


def offset(x, y):
    return (screen_width / 2.0) + x, (screen_height / 2) - y


def inverse_offset(x_transformed, y_transformed):
    x = x_transformed - (screen_width / 2.0)
    y = (screen_height / 2) - y_transformed
    return x, y


def plot_robot(screen, robot):

        x0 = 0
        y0 = 0
        x0, y0 = scale(x0), scale(y0)
        x0, y0 = offset(x0, y0)

        x1 = robot._BASE_ARM_LENGTH * np.cos(robot._joint_position[0])
        y1 = robot._BASE_ARM_LENGTH * np.sin(robot._joint_position[0])
        x1, y1 = scale(x1), scale(y1)
        x1, y1 = offset(x1, y1)

        x2 = robot._cartesian_position[0] - robot._TCP_OFFSET[0]
        y2 = robot._cartesian_position[1] - robot._TCP_OFFSET[1]
        x2, y2 = scale(x2), scale(y2)
        x2, y2 = offset(x2, y2)

        x3 = robot._BASE_ARM_LENGTH * np.cos(
            robot._joint_position[1]) + robot._MOTOR_DISTANCE
        y3 = robot._BASE_ARM_LENGTH * np.sin(robot._joint_position[1])
        x3, y3 = scale(x3), scale(y3)
        x3, y3 = offset(x3, y3)

        x4 = robot._MOTOR_DISTANCE
        y4 = 0
        x4, y4 = scale(x4), scale(y4)
        x4, y4 = offset(x4, y4)

        x_tcp = robot._cartesian_position[0]
        y_tcp = robot._cartesian_position[1]
        x_tcp, y_tcp = scale(x_tcp), scale(y_tcp)
        x_tcp, y_tcp = offset(x_tcp, y_tcp)

        pygame.draw.line(screen, (255, 255, 255), (x0, y0), (x1, y1), 5)
        pygame.draw.line(screen, (255, 255, 255), (x1, y1), (x2, y2), 5)
        pygame.draw.line(screen, (255, 255, 255), (x2, y2), (x3, y3), 5)
        pygame.draw.line(screen, (255, 255, 255), (x3, y3), (x4, y4), 5)
        pygame.draw.line(screen, (255, 255, 255), (x3, y3), (x_tcp, y_tcp), 5)
        pygame.draw.circle(
            screen, (255, 255, 255), (x0, y0),
            scale(np.abs(robot._BASE_ARM_LENGTH - robot._LINK_ARM_LENGTH)), 1)
        pygame.draw.circle(
            screen, (255, 255, 255), (x0, y0),
            scale((robot._BASE_ARM_LENGTH + robot._LINK_ARM_LENGTH)), 1)
        pygame.draw.circle(
            screen, (255, 255, 255), (x4, y4),
            scale((robot._BASE_ARM_LENGTH + robot._LINK_ARM_LENGTH)), 1)
        pygame.draw.circle(
            screen, (255, 255, 255), (x4, y4),
            scale(np.abs(robot._BASE_ARM_LENGTH - robot._LINK_ARM_LENGTH)), 1)


def visualization(stop_event, telemetry_sharer):
    # Initialize Pygame
    pygame.init()

    # Colors
    black = (0, 0, 0)
    white = (255, 255, 255)
    orange = (255, 162, 0)

    # Set up the display
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption('Robot Simulator')

    # Create a canvas surface
    canvas = pygame.Surface(screen.get_size())
    canvas.fill(black)

    # Robot
    initial_joint_position = np.array([2.5, 0.6])
    parallel_robot = Robot(initial_joint_position)
    previous_telemetry = telemetry_sharer.get_telemetry()
    telemetry = copy(previous_telemetry)

    # Clock to control the frame rate
    time_manager = TimeManager(0.005)

    # Main game loop
    while not stop_event.is_set():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        previous_telemetry = copy(telemetry)
        telemetry = telemetry_sharer.get_telemetry()
        if telemetry is not None:
            parallel_robot.set_cartesian_position(telemetry.cartesian_position)
            if telemetry.drawing:
                last_pos = previous_telemetry.cartesian_position
                current_pos = telemetry.cartesian_position
                # print(last_pos, current_pos)
                last_pos = scale(last_pos[0]), scale(last_pos[1])
                current_pos = scale(current_pos[0]), scale(current_pos[1])
                last_pos = offset(last_pos[0], last_pos[1])
                current_pos = offset(current_pos[0], current_pos[1])
                # print(last_pos, current_pos)
                pygame.draw.line(canvas, orange, last_pos, current_pos, 1)

        # Blit the canvas onto the screen
        screen.blit(canvas, (0, 0))

        # Draw the robot
        plot_robot(screen, parallel_robot)

        # Update the display
        pygame.display.flip()

        # Cap the frame rate
        time_manager.adaptive_sleep()


if __name__ == "__main__":
    visualization()
