import sys
import pygame
import numpy as np

from robot import Robot

# Screen dimensions
screen_width = 640
screen_height = 480

pixel_per_metre = 2000


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


def main():
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
    last_pos = None  # Store the last position of the mouse

    # Robot
    parallel_robot = Robot(np.array([np.pi / 2, np.pi / 2]))
    print(parallel_robot.get_cartesian_position())

    # Clock to control the frame rate
    clock = pygame.time.Clock()

    # Main game loop
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                drawing = True
                last_pos = pygame.mouse.get_pos()  # Start drawing
            elif event.type == pygame.MOUSEBUTTONUP:
                drawing = False
                last_pos = None  # Reset last position

        if drawing and last_pos:
            # Draw line on canvas
            current_pos = pygame.mouse.get_pos()
            pygame.draw.line(canvas, white, last_pos, current_pos, 1)
            last_pos = current_pos  # Update the last position for continuous drawing

        # Move robot
        mouse_x, mouse_y = pygame.mouse.get_pos()

        mouse_x, mouse_y = inverse_offset(mouse_x, mouse_y)
        mouse_x, mouse_y = inverse_scale(mouse_x), inverse_scale(mouse_y)

        parallel_robot._joint_position = parallel_robot.inverse_kinematics(
            np.array([mouse_x, mouse_y]))
        parallel_robot._cartesian_position = np.array([mouse_x, mouse_y])

        # Blit the canvas onto the screen
        screen.blit(canvas, (0, 0))

        # Draw the robot
        parallel_robot.plot_robot(screen, scale, offset)

        # Update the display
        pygame.display.flip()

        # Cap the frame rate
        clock.tick(60)


if __name__ == "__main__":
    main()
