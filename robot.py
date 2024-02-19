import numpy as np
import pygame


class Robot:

    # Units in metres
    _MOTOR_DISTANCE = 0.05
    _BASE_ARM_LENGTH = 0.05
    _LINK_ARM_LENGTH = 0.08
    _TCP_OFFSET = [0, 0]

    def __init__(self, initial_joint_position: np.ndarray):
        self._joint_position = initial_joint_position
        self._cartesian_position = self._solve_forward_kinematics(
            self._joint_position)[1]

    def get_joint_position(self):
        return self._joint_position

    def get_cartesian_position(self):
        return self._cartesian_position

    def _solve_forward_kinematics(self, joint_position: np.ndarray):

        theta1 = joint_position[0]
        theta4 = joint_position[1]

        la = self._BASE_ARM_LENGTH
        lb = self._LINK_ARM_LENGTH
        lc = self._MOTOR_DISTANCE

        # Constants based on the equations given
        E = 2 * la * (lc + la * (np.cos(theta4) - np.cos(theta1)))
        F = 2 * la * lb * (np.sin(theta4) - np.sin(theta1))
        G = lc**2 + 2 * la**2 + 2 * lc * la * np.cos(
            theta4) - 2 * lc * la * np.cos(theta1) - 2 * la**2 * np.cos(
                theta4 - theta1)

        # Calculate x_p and y_p for both solutions
        positive_arctan = 2 * np.arctan(
            (-F + np.sqrt(E**2 + F**2 - G**2)) / (G - E))
        negative_arctan = 2 * np.arctan(
            (-F - np.sqrt(E**2 + F**2 - G**2)) / (G - E))
        x_p1 = lc + la * np.cos(theta4) + lb * np.cos(positive_arctan)
        x_p2 = lc + la * np.cos(theta4) + lb * np.cos(negative_arctan)
        y_p1 = la * np.sin(theta4) + lb * np.sin(positive_arctan)
        y_p2 = la * np.sin(theta4) + lb * np.sin(negative_arctan)

        # Apply offset
        x_p1 += self._TCP_OFFSET[0]
        x_p2 += self._TCP_OFFSET[0]
        y_p1 += self._TCP_OFFSET[1]
        y_p2 += self._TCP_OFFSET[1]

        return np.array([[x_p1, y_p1], [x_p2, y_p2]])

    def _solve_inverse_kinematics(self, cartesian_position: np.ndarray):

        x = cartesian_position[0]
        y = cartesian_position[1]

        la = self._BASE_ARM_LENGTH
        lb = self._LINK_ARM_LENGTH
        lc = self._MOTOR_DISTANCE

        # Apply offset
        x -= self._TCP_OFFSET[0]
        y -= self._TCP_OFFSET[1]

        # Constants based on the equations given for θ1
        E1 = -2 * la * x
        F1 = -2 * la * y
        G1 = la**2 - lb**2 + x**2 + y**2

        # Constants based on the equations given for θ4
        E4 = 2 * la * (-x + lc)
        F4 = -2 * la * y
        G4 = lc**2 + la**2 - lb**2 + x**2 + y**2 - 2 * lc * x

        # Calculate theta1 for both solutions
        theta1_pos = 2 * np.arctan(
            (-F1 + np.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1))
        theta1_neg = 2 * np.arctan(
            (-F1 - np.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1))

        # Calculate theta4 for both solutions
        theta4_pos = 2 * np.arctan(
            (-F4 + np.sqrt(E4**2 + F4**2 - G4**2)) / (G4 - E4))
        theta4_neg = 2 * np.arctan(
            (-F4 - np.sqrt(E4**2 + F4**2 - G4**2)) / (G4 - E4))

        # Return the four possible combinations of solutions
        return np.array([[theta1_pos, theta4_neg]])
        # return np.array([[theta1_pos, theta4_pos], [theta1_pos, theta4_neg],
        #                  [theta1_neg, theta4_pos], [theta1_neg, theta4_neg]])

    def forward_kinematics(self, joint_position: np.ndarray):

        solutions = self._solve_forward_kinematics(joint_position)

        # Calculate solution distances from the current position
        distances = np.linalg.norm(solutions - self._cartesian_position,
                                   axis=1)

        # Find the index of the closest solution
        min_index = np.argmin(distances)

        # Return the closest solution
        return solutions[min_index]

    def inverse_kinematics(self, cartesian_position: np.ndarray):

        solutions = self._solve_inverse_kinematics(cartesian_position)

        # Calculate solution distances from the current position
        distances = np.linalg.norm(solutions - self._joint_position, axis=1)

        # Find the index of the closest solution
        min_index = np.argmin(distances)

        # Return the closest solution
        return solutions[min_index]

    def plot_robot(self, screen, scale, offset):

        x0 = 0
        y0 = 0
        x0, y0 = scale(x0), scale(y0)
        x0, y0 = offset(x0, y0)

        x1 = self._BASE_ARM_LENGTH * np.cos(self._joint_position[0])
        y1 = self._BASE_ARM_LENGTH * np.sin(self._joint_position[0])
        x1, y1 = scale(x1), scale(y1)
        x1, y1 = offset(x1, y1)

        x2 = self._cartesian_position[0] - self._TCP_OFFSET[0]
        y2 = self._cartesian_position[1] - self._TCP_OFFSET[1]
        x2, y2 = scale(x2), scale(y2)
        x2, y2 = offset(x2, y2)

        x3 = self._BASE_ARM_LENGTH * np.cos(
            self._joint_position[1]) + self._MOTOR_DISTANCE
        y3 = self._BASE_ARM_LENGTH * np.sin(self._joint_position[1])
        x3, y3 = scale(x3), scale(y3)
        x3, y3 = offset(x3, y3)

        x4 = self._MOTOR_DISTANCE
        y4 = 0
        x4, y4 = scale(x4), scale(y4)
        x4, y4 = offset(x4, y4)

        x_tcp = self._cartesian_position[0]
        y_tcp = self._cartesian_position[1]
        x_tcp, y_tcp = scale(x_tcp), scale(y_tcp)
        x_tcp, y_tcp = offset(x_tcp, y_tcp)

        pygame.draw.line(screen, (255, 255, 255), (x0, y0), (x1, y1), 5)
        pygame.draw.line(screen, (255, 255, 255), (x1, y1), (x2, y2), 5)
        pygame.draw.line(screen, (255, 255, 255), (x2, y2), (x3, y3), 5)
        pygame.draw.line(screen, (255, 255, 255), (x3, y3), (x4, y4), 5)
        pygame.draw.line(screen, (255, 255, 255), (x3, y3), (x_tcp, y_tcp), 5)
        pygame.draw.circle(
            screen, (255, 255, 255), (x0, y0),
            np.abs(self._BASE_ARM_LENGTH - self._LINK_ARM_LENGTH) * 2000, 1)
        pygame.draw.circle(screen, (255, 255, 255), (x0, y0),
                           (self._BASE_ARM_LENGTH + self._LINK_ARM_LENGTH) *
                           2000, 1)
        pygame.draw.circle(screen, (255, 255, 255), (x4, y4),
                           (self._BASE_ARM_LENGTH + self._LINK_ARM_LENGTH) *
                           2000, 1)
        pygame.draw.circle(
            screen, (255, 255, 255), (x4, y4),
            np.abs(self._BASE_ARM_LENGTH - self._LINK_ARM_LENGTH) * 2000, 1)
