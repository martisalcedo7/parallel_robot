#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>
#include <stdint.h>

// Robot geometry
#define MOTOR_DISTANCE 0.10f
#define BASE_ARM_LENGTH 0.05f
#define LINK_ARM_LENGTH 0.05f

// Robot state
typedef struct {
  float joint_position[2];
  float cartesian_position[2];
  bool drawing;
  bool calibrated;
} Robot;

void robot_init(Robot *robot, const float initial_joint_position[2]);
void Robot_set_drawing(Robot *robot, bool draw);
void Robot_set_joint_position(Robot *robot, const float new_joint_position[2]);
void Robot_cartesian_position(Robot *robot,
                              const float new_cartesian_position[2]);
bool Robot_is_cartesian_position_in_workspace(
    Robot *robot, const float cartesian_position[2]);
void Robot_forward_kinematics(Robot *robot, const float joint_position[2],
                              float result[2]);
void Robot_inverse_kinematics(Robot *robot, const float cartesian_position[2],
                              float result[2]);

#endif