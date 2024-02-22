import numpy as np

def control_client(telemetry_sharer, command_sharer):

    while True:
        coordinates = input("Give coordinates: ")
        x, y = coordinates.replace(" ", "").split(",")
        x = float(x)
        y = float(y)
        item = np.array([x, y])
        command_sharer.add_command(item)


