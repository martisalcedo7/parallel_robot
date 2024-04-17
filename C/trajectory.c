#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "trajectory.h"

void Trajectory_init(Trajectory *trajectory, const float initial_position,
                     const float final_position, const float max_velocity,
                     const float acceleration) {
  trajectory->active = false;

  trajectory->initial_position = initial_position;
  trajectory->final_position = final_position;
  trajectory->current_position = trajectory->initial_position;
  trajectory->current_time = 0.0f;

  float position_increment =
      trajectory->final_position - trajectory->initial_position;

  trajectory->acceleration = copysignf(acceleration, position_increment);

  float velocity =
      fminf(sqrtf(position_increment * trajectory->acceleration), max_velocity);
  trajectory->max_velocity = copysignf(velocity, position_increment);

  trajectory->t1 = trajectory->max_velocity / trajectory->acceleration;
  trajectory->t2 = position_increment / trajectory->max_velocity;
  trajectory->t3 = trajectory->t1 + trajectory->t2;

  trajectory->active = true;
}

void Trajectory_update_to_next_position(Trajectory *trajectory,
                                        const float timer_period) {
  if (!trajectory->active) {
    return;
  }

  trajectory->current_time += timer_period;

  if (trajectory->current_time <= trajectory->t1) {
    trajectory->current_position = 0.5 * trajectory->acceleration *
                                   trajectory->current_time *
                                   trajectory->current_time;

  } else if (trajectory->current_time > trajectory->t1 &&
             trajectory->current_time <= trajectory->t2) {
    trajectory->current_position =
        trajectory->max_velocity * (trajectory->current_time - trajectory->t1) +
        0.5 * trajectory->acceleration * trajectory->t1 * trajectory->t1;

  } else if (trajectory->current_time > trajectory->t2 &&
             trajectory->current_time <= trajectory->t3) {
    trajectory->current_position =
        trajectory->max_velocity * (trajectory->current_time - trajectory->t1) +
        0.5 * trajectory->acceleration * trajectory->t1 * trajectory->t1 -
        0.5 * trajectory->acceleration *
            (trajectory->current_time - trajectory->t2) *
            (trajectory->current_time - trajectory->t2);
  } else {
    trajectory->active = false;
  }
}

float Trajectory_get_current_position(Trajectory *trajectory) {
  return trajectory->current_position;
}

bool Trajectory_is_active(Trajectory *trajectory) { return trajectory->active; }