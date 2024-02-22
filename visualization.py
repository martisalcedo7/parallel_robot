import sys
import pygame
import numpy as np

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


def visualization(telemetry_sharer):
    # Initialize Pygame
    pygame.init()

    # Colors
    black = (0, 0, 0)
    white = (255, 255, 255)

    # Set up the display
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption('Robot Simulator')

    # Create a canvas surface
    canvas = pygame.Surface(screen.get_size())
    canvas.fill(black)

    # Flag to track mouse button state
    drawing = False

    # Robot
    initial_joint_position = np.array([2.5, 0.6])
    parallel_robot = Robot(initial_joint_position)

    # Clock to control the frame rate
    time_manager = TimeManager(0.005)

    # Main game loop
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        
        cartesian_position = telemetry_sharer.get_telemetry()
        if cartesian_position is not None:
            parallel_robot.set_cartesian_position(cartesian_position)

        # Blit the canvas onto the screen
        screen.blit(canvas, (0, 0))

        # Draw the robot
        parallel_robot.plot_robot(screen, scale, offset)

        # Update the display
        pygame.display.flip()

        # Cap the frame rate
        time_manager.adaptive_sleep()


if __name__ == "__main__":
    visualization()
