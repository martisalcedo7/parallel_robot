import numpy as np
from dataclasses import dataclass
from enum import Enum
from time import sleep

@dataclass
class Command:
    cartesian_position: np.ndarray
    drawing: bool


def control_client(stop_event, telemetry_sharer, command_sharer):

    r = 0.02
    command = Command(np.array([0.02619328 + r, 0.07476641]), False)
    command_sharer.add_command(command)
    for angle in range(360):
        x = r*np.cos(angle*2*np.pi/360) + 0.02619328
        y = r*np.sin(angle*2*np.pi/360) + 0.07476641
        command = Command(np.array([float(x), float(y)]), True)
        command_sharer.add_command(command)
    sleep(10)
    stop_event.set()
    # while not stop_event.is_set():
    #     telemetry = telemetry_sharer.get_telemetry()
    #     print(f"Current telemetry: {telemetry}")
    #     coordinates = input("Give command [cartesian_x,cartesian_y,drawing]: ")
    #     if coordinates == "exit":
    #         stop_event.set()
    #         break
    #     x, y, draw = coordinates.replace(" ", "").split(",")
    #     draw = True if draw == "True" else False
    #     command = Command(np.array([float(x), float(y)]), bool(draw))
    #     command_sharer.add_command(command)
