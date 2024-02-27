import numpy as np


def constant_velocity(initial_position, final_position, max_velocity,
                      sampling_time):
    position_increment = abs(final_position - initial_position)
    max_position_increment = max(position_increment)
    time_to_destination = max_position_increment / max_velocity
    if time_to_destination <= sampling_time:
        return [final_position]

    # Calculate component velocities
    vx = (final_position[0] - initial_position[0]) / time_to_destination
    vy = (final_position[1] - initial_position[1]) / time_to_destination

    # Generate the trajectory
    num_samples = int(time_to_destination / sampling_time) + 1
    trajectory = []
    for i in range(num_samples - 1):
        t = (i + 1) * sampling_time
        x = initial_position[0] + vx * t
        y = initial_position[1] + vy * t
        trajectory.append(np.array([x, y]))

    return trajectory


if __name__ == "__main__":
    print(constant_velocity(np.array([0, 0]), np.array([10, 5]), 5, 1))
