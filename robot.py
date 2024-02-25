import numpy as np
import pygame
from dataclasses import dataclass


@dataclass
class Telemetry:
    joint_position: np.ndarray
    cartesian_position: np.ndarray
    drawing: bool


class Robot:

    # Units in metres
    _MOTOR_DISTANCE = 0.05
    _BASE_ARM_LENGTH = 0.05
    _LINK_ARM_LENGTH = 0.08
    _TCP_OFFSET = [0, 0]

    def __init__(self, initial_joint_position: np.ndarray):
        self._joint_position = initial_joint_position
        self._cartesian_position = self.forward_kinematics(
            self._joint_position)
        self._drawing = False

    def get_telemetry(self):
        return Telemetry(self._joint_position, self._cartesian_position,
                         self._drawing)

    def set_drawing(self, drawing):
        self._drawing = drawing

    def set_joint_position(self, joint_position):
        self._joint_position = joint_position
        self._cartesian_position = self.forward_kinematics(
            self._joint_position)

    def set_cartesian_position(self, cartesian_position):
        self._cartesian_position = cartesian_position
        self._joint_position = self.inverse_kinematics(
            self._cartesian_position)

    def in_workspace(self, cartesian_position):
        in_area_1 = cartesian_position[0]**2 + cartesian_position[1]**2 >= (
            self._BASE_ARM_LENGTH - self._LINK_ARM_LENGTH)**2
        in_area_2 = (cartesian_position[0] -
                     self._MOTOR_DISTANCE)**2 + cartesian_position[1]**2 >= (
                         self._BASE_ARM_LENGTH - self._LINK_ARM_LENGTH)**2
        in_area_3 = cartesian_position[0]**2 + cartesian_position[1]**2 <= (
            self._BASE_ARM_LENGTH + self._LINK_ARM_LENGTH)**2
        in_area_4 = (cartesian_position[0] -
                     self._MOTOR_DISTANCE)**2 + cartesian_position[1]**2 <= (
                         self._BASE_ARM_LENGTH + self._LINK_ARM_LENGTH)**2

        return in_area_1 and in_area_2 and in_area_3 and in_area_4

    def forward_kinematics(self, joint_position: np.ndarray):

        theta1 = joint_position[0]
        theta4 = joint_position[1]

        la = self._BASE_ARM_LENGTH
        lb = self._LINK_ARM_LENGTH
        lc = self._MOTOR_DISTANCE

        # Constants based on the equations given
        E = 2 * lb * (lc + la * (np.cos(theta4) - np.cos(theta1)))
        F = 2 * la * lb * (np.sin(theta4) - np.sin(theta1))
        G = lc**2 + 2 * la**2 + 2 * lc * la * np.cos(
            theta4) - 2 * lc * la * np.cos(theta1) - 2 * la**2 * np.cos(
                theta4 - theta1)

        # Calculate x_p and y_p for both solutions
        # positive_arctan = 2 * np.arctan(
        #     (-F + np.sqrt(E**2 + F**2 - G**2)) / (G - E))
        negative_arctan = 2 * np.arctan(
            (-F - np.sqrt(E**2 + F**2 - G**2)) / (G - E))
        # x_p1 = lc + la * np.cos(theta4) + lb * np.cos(positive_arctan)
        x_p2 = lc + la * np.cos(theta4) + lb * np.cos(negative_arctan)
        # y_p1 = la * np.sin(theta4) + lb * np.sin(positive_arctan)
        y_p2 = la * np.sin(theta4) + lb * np.sin(negative_arctan)

        # Apply offset
        # x_p1 += self._TCP_OFFSET[0]
        x_p2 += self._TCP_OFFSET[0]
        # y_p1 += self._TCP_OFFSET[1]
        y_p2 += self._TCP_OFFSET[1]
        return np.array([x_p2, y_p2])
        # return np.array([[x_p1, y_p1], [x_p2, y_p2]])

    def inverse_kinematics(self, cartesian_position: np.ndarray):

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
        # theta1_neg = 2 * np.arctan(
        #     (-F1 - np.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1))

        # Calculate theta4 for both solutions
        # theta4_pos = 2 * np.arctan(
        #     (-F4 + np.sqrt(E4**2 + F4**2 - G4**2)) / (G4 - E4))
        theta4_neg = 2 * np.arctan(
            (-F4 - np.sqrt(E4**2 + F4**2 - G4**2)) / (G4 - E4))

        # Return the four possible combinations of solutions
        return np.array([theta1_pos, theta4_neg])
        # return np.array([[theta1_pos, theta4_pos], [theta1_pos, theta4_neg],
        #                  [theta1_neg, theta4_pos], [theta1_neg, theta4_neg]])

