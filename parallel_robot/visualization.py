import sys
import pygame
import numpy as np

from .tools import TimeManager

# Colors
black = (0, 0, 0)
white = (255, 255, 255)
orange = (255, 162, 0)
red = (255, 0, 0)
green = (0, 255, 0)


class Visualization:

    def __init__(self, stop_event, telemetry_sharer, robot_configuration):
        self._screen_width = 800
        self._screen_height = 600
        self._pixel_per_metre = 2000

        self._stop_event = stop_event
        self._telemetry_sharer = telemetry_sharer
        self._robot_configuration = robot_configuration

    def scale(self, value):
        """Scales a value based on pixels per metre."""
        return value * self._pixel_per_metre

    def robot_to_screen_coordinates(self, x, y):
        """Converts robot coordinates to screen coordinates."""
        center_offset = (self._screen_width / 2.0) - (
            self.scale(self._robot_configuration.motor_distance) / 2.0)
        return center_offset + self.scale(x), (self._screen_height /
                                               2.0) - self.scale(y)

    def draw_robot(self, screen, telemetry):
        """Draws the robot based on telemetry data."""
        points = [
            (0, 0),  # x0, y0
            (
                self._robot_configuration.base_arm_length *
                np.cos(telemetry.joint_position[0]),  # x1
                self._robot_configuration.base_arm_length *
                np.sin(telemetry.joint_position[0])),  # y1
            (
                telemetry.cartesian_position[0] -
                self._robot_configuration.tcp_offset[0],  # x2
                telemetry.cartesian_position[1] -
                self._robot_configuration.tcp_offset[1]),  # y2
            (
                self._robot_configuration.base_arm_length *
                np.cos(telemetry.joint_position[1]) +  # x3
                self._robot_configuration.motor_distance,
                self._robot_configuration.base_arm_length *
                np.sin(telemetry.joint_position[1])),  # y3
            (self._robot_configuration.motor_distance, 0)  # x4, y4
        ]

        # Convert all robot coordinates to screen coordinates
        screen_points = [
            self.robot_to_screen_coordinates(x, y) for x, y in points
        ]
        x_tcp, y_tcp = self.robot_to_screen_coordinates(
            *telemetry.cartesian_position)

        # Draw lines
        white = (255, 255, 255)
        for i in range(len(screen_points) - 1):
            pygame.draw.line(screen, white, screen_points[i],
                             screen_points[i + 1], 5)
        pygame.draw.line(screen, white, screen_points[2], (x_tcp, y_tcp), 5)

        # Draw circles for arm lengths
        for x4, y4 in [screen_points[0], screen_points[-1]]:
            pygame.draw.circle(
                screen, white, (x4, y4),
                self.scale(
                    np.abs(self._robot_configuration.base_arm_length -
                           self._robot_configuration.link_arm_length)), 1)
            pygame.draw.circle(
                screen, white, (x4, y4),
                self.scale(self._robot_configuration.base_arm_length +
                           self._robot_configuration.link_arm_length), 1)

    def draw_axes(self, screen, font):
        """Draws coordinate axes on the screen."""
        red = (255, 0, 0)
        green = (0, 255, 0)
        center_x, center_y = self.robot_to_screen_coordinates(0, 0)

        # Draw arrows and labels for axes
        arrow_length = 20
        pygame.draw.line(screen, red, (center_x, center_y),
                         (center_x + arrow_length, center_y), 2)
        pygame.draw.line(screen, green, (center_x, center_y),
                         (center_x, center_y - arrow_length), 2)

        x_label = font.render('x', True, red)
        y_label = font.render('y', True, green)
        screen.blit(x_label, (center_x + arrow_length + 5, center_y))
        screen.blit(y_label, (center_x, center_y - arrow_length - 20))

    def main_loop(self):
        """Main loop for running the visualization."""
        pygame.init()

        screen = pygame.display.set_mode(
            (self._screen_width, self._screen_height))

        pygame.display.set_caption('Parallel Robot Simulator')

        font = pygame.font.SysFont(None, 24)

        canvas = pygame.Surface(screen.get_size())
        canvas.fill(black)  # Black color for canvas background

        time_manager = TimeManager(0.001)

        previous_telemetry = None  # Initialize previous_telemetry

        while not self._stop_event.is_set():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            telemetry = self._telemetry_sharer.get_telemetry()
            if telemetry and previous_telemetry and telemetry.drawing:
                last_pos = self.robot_to_screen_coordinates(
                    *previous_telemetry.cartesian_position)
                current_pos = self.robot_to_screen_coordinates(
                    *telemetry.cartesian_position)
                pygame.draw.line(canvas, orange, last_pos, current_pos, 1)

            previous_telemetry = telemetry  # Update previous_telemetry for the next iteration

            screen.blit(canvas, (0, 0))
            self.draw_axes(screen, font)
            self.draw_robot(screen, telemetry)
            pygame.display.flip()

            time_manager.adaptive_sleep()
