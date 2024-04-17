#include "robot.h"
#include <math.h>
#include <stdbool.h>

void Robot_init(Robot *robot, float initial_joint_position[2]) {
  Robot_set_joint_position(robot, initial_joint_position);
  Robot_set_drawing(robot, 0);
}

void Robot_set_drawing(Robot *robot, bool draw) { robot->drawing = draw; }

void Robot_set_joint_position(Robot *robot, const float new_joint_position[2]) {
  robot->joint_position[0] = new_joint_position[0];
  robot->joint_position[1] = new_joint_position[1];
  float cartesian_position[2];
  Robot_forward_kinematics(robot, robot->joint_position, cartesian_position);
  robot->cartesian_position[0] = cartesian_position[0];
  robot->cartesian_position[1] = cartesian_position[1];
}

void Robot_set_cartesian_position(Robot *robot,
                                  const float new_cartesian_position[2]) {
  robot->cartesian_position[0] = new_cartesian_position[0];
  robot->cartesian_position[1] = new_cartesian_position[1];
  float joint_position[2];
  Robot_inverse_kinematics(robot, robot->cartesian_position, joint_position);
  robot->joint_position[0] = joint_position[0];
  robot->joint_position[1] = joint_position[1];
}

bool Robot_is_cartesian_position_in_workspace(
    Robot *robot, const float cartesian_position[2]) {
  float x = cartesian_position[0];
  float y = cartesian_position[1];
  bool in_area_1 =
      (pow(x, 2) + pow(y, 2)) >= pow(BASE_ARM_LENGTH - LINK_ARM_LENGTH, 2);
  bool in_area_2 = (pow(x - MOTOR_DISTANCE, 2) + pow(y, 2)) >=
                   pow(BASE_ARM_LENGTH - LINK_ARM_LENGTH, 2);
  bool in_area_3 =
      (pow(x, 2) + pow(y, 2)) <= pow(BASE_ARM_LENGTH + LINK_ARM_LENGTH, 2);
  bool in_area_4 = (pow(x - MOTOR_DISTANCE, 2) + pow(y, 2)) <=
                   pow(BASE_ARM_LENGTH + LINK_ARM_LENGTH, 2);

  return in_area_1 && in_area_2 && in_area_3 && in_area_4;
}

void Robot_forward_kinematics(Robot *robot, const float joint_position[2],
                              float result[2]) {
  float theta1 = joint_position[0];
  float theta4 = joint_position[1];
  float la = BASE_ARM_LENGTH;
  float lb = LINK_ARM_LENGTH;
  float lc = MOTOR_DISTANCE;

  float E = 2 * lb * (lc + la * (cos(theta4) - cos(theta1)));
  float F = 2 * la * lb * (sin(theta4) - sin(theta1));
  float G = pow(lc, 2) + 2 * pow(la, 2) + 2 * lc * la * cos(theta4) -
            2 * lc * la * cos(theta1) - 2 * pow(la, 2) * cos(theta4 - theta1);

  float negative_atan =
      2 * atan((-F - sqrt(pow(E, 2) + pow(F, 2) - pow(G, 2))) / (G - E));
  result[0] = lc + la * cos(theta4) + lb * cos(negative_atan);
  result[1] = la * sin(theta4) + lb * sin(negative_atan);
}

void Robot_inverse_kinematics(Robot *robot, const float cartesian_position[2],
                              float result[2]) {
  float x = cartesian_position[0];
  float y = cartesian_position[1];
  float la = BASE_ARM_LENGTH;
  float lb = LINK_ARM_LENGTH;
  float lc = MOTOR_DISTANCE;

  float E1 = -2 * la * x;
  float F1 = -2 * la * y;
  float G1 = pow(la, 2) - pow(lb, 2) + pow(x, 2) + pow(y, 2);

  float E4 = 2 * la * (-x + lc);
  float F4 = -2 * la * y;
  float G4 =
      pow(lc, 2) + pow(la, 2) - pow(lb, 2) + pow(x, 2) + pow(y, 2) - 2 * lc * x;

  result[0] = 2 * atan((-F1 + sqrt(pow(E1, 2) + pow(F1, 2) - pow(G1, 2))) /
                       (G1 - E1)); // theta1_pos
  result[1] = 2 * atan((-F4 - sqrt(pow(E4, 2) + pow(F4, 2) - pow(G4, 2))) /
                       (G4 - E4)); // theta4_neg
}
