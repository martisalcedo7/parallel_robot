from enum import Enum
import threading
import queue

class Command(Enum):
    MOVE_JOINT=1
    MOVE_CARTESIAN=1

class TelemetrySharer:
    def __init__(self):
        self._telemetry = None
        self._lock = threading.Lock()

    def update_telemetry(self, new_telemetry):
        with self._lock:
            self._telemetry = new_telemetry

    def get_telemetry(self):
        with self._lock:
            telemetry = self._telemetry
        return telemetry

class CommandSharer:
    def __init__(self):
        self._commands_queue = queue.Queue()

    def add_command(self, command):
        try:
            self._commands_queue.put(command, block=False)
        except queue.Full:
            return False
        return True
    
    def get_command(self):
        try:
            command = self._commands_queue.get(block=False)
        except queue.Empty:
            return None
        return command

if __name__ == "__main__":

    telemetry_sharer = TelemetrySharer()
    command_sharer = CommandSharer()

    def robot():
        while True:
            item = command_sharer.get_command()
            if item:
                print(item)

    # Turn-on the worker thread.
    threading.Thread(target=robot, daemon=True).start()

    # Send thirty task requests to the worker.
    running = True
    while running:
        command_type = int(input("\nGive command type: "))
        coordinates = input("Give coordinates: ")
        x, y = coordinates.replace(" ", "").split(",")
        item = {'command': command_type, 'coordinates': [x, y]}
        command_sharer.add_command(item)

    # Block until all tasks are done.
    robot.join()
