from .simulation import simulation
from .visualization import Visualization
from .tools import TelemetrySharer, CommandSharer, RobotConfiguration

import threading
import numpy as np
import signal
import sys
from time import sleep

class ParallelRobotSimulation:

    def __init__(self,
                 motor_distance: float,
                 base_arm_length: float,
                 link_arm_length: float,
                 tcp_offset: np.ndarray = np.array([0.0, 0.0]),
                 initial_joint_position: np.ndarray = np.array(
                     [np.pi / 2, np.pi / 2]),
                 visualization=True):

        self._robot_configuration = RobotConfiguration(motor_distance,
                                                       base_arm_length,
                                                       link_arm_length,
                                                       tcp_offset,
                                                       initial_joint_position)

        self._telemetry_sharer = TelemetrySharer()
        self._command_sharer = CommandSharer()

        self._stop_event = threading.Event()
        self._visualization_thread = None
        self._simulation_thread = None

        self._signal = signal.signal(signal.SIGINT, self._signal_handler)

        self._visualization = None
        if visualization:
            self._visualization = Visualization(
                self._stop_event,
                self._telemetry_sharer,
                self._robot_configuration,
            )

    def _signal_handler(self, _sig, _frame):
        print("Ctrl+C exiting the program.")
        self.stop()

    def start(self):
        if self._visualization:
            self._visualization_thread = threading.Thread(
                target=self._visualization.main_loop, daemon=True, args=())

            self._visualization_thread.start()

        self._simulation_thread = threading.Thread(
            target=simulation,
            daemon=True,
            args=(
                self._stop_event,
                self._telemetry_sharer,
                self._command_sharer,
                self._robot_configuration,
            ))

        self._simulation_thread.start()

    def stop(self):
        self._stop_event.set()

        if self._visualization:
            self._visualization_thread.join()

        self._simulation_thread.join()

        sys.exit(0)
    
    def wait_until_no_commands(self):
        while self._command_sharer.get_number_of_commands() > 0:
            sleep(0.5)
        self.stop()

    def add_command(self, command):
        self._command_sharer.add_command(command)

    def get_telemetry(self):
        return self._telemetry_sharer.get_telemetry()
    
    def get_robot_configuration(self):
        return self._robot_configuration
