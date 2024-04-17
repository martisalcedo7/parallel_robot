#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float initial_position;
  float final_position;
  float current_position;
  float current_time;
  float max_velocity;
  float acceleration;
  float t1;
  float t2;
  float t3;
  bool active;
} Trajectory;

void Trajectory_init(Trajectory *trajectory, const float initial_position,
                     const float final_position, const float max_velocity,
                     const float acceleration);

void Trajectory_update_to_next_position(Trajectory *trajectory,
                                        const float timer_period);

float Trajectory_get_current_position(Trajectory *trajectory);

bool Trajectory_is_active(Trajectory *trajectory);

#endif