import sys
import pygame
import numpy as np

from robot import Robot

# Screen dimensions
screen_width = 640
screen_height = 480

pixel_per_metre = 2000


def offset_and_scale(x, y):

    return (screen_width/2.0) + (x * pixel_per_metre), (screen_height/2) - (y * pixel_per_metre)

def inverse_offset_and_scale(x_transformed, y_transformed):
    x = (x_transformed - (screen_width / 2.0)) / pixel_per_metre
    y = ((screen_height/2) - y_transformed) / pixel_per_metre
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

        # Move robot
        mouse_x, mouse_y = pygame.mouse.get_pos()

        mouse_x, mouse_y = inverse_offset_and_scale(mouse_x, mouse_y)

        parallel_robot._joint_position = parallel_robot.inverse_kinematics(np.array([mouse_x, mouse_y]))
        parallel_robot._cartesian_position = np.array([mouse_x, mouse_y])


        # Fill the screen with black
        screen.fill(black)

        # Draw the ball
        parallel_robot.plot_robot(screen, offset_and_scale)

        # Update the display
        pygame.display.flip()

        # Cap the frame rate
        clock.tick(60)


if __name__ == "__main__":
    main()
