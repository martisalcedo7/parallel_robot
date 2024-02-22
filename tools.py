import time
import threading
import queue


class TimeManager:

    def __init__(self, sampling_time: float) -> None:
        self._sampling_time = sampling_time
        self._initial_time = time.time()
        self._previous_time = self._initial_time

    def adaptive_sleep(self):
        current_time = time.time()
        time_increment = current_time - self._previous_time
        sleep_time = max(0, self._sampling_time - time_increment)
        time.sleep(sleep_time)
        self._previous_time = current_time + sleep_time

    def get_time(self):
        return time.time() - self._initial_time


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
