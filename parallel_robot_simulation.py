from simulation import simulation
from visualization import visualization
from tools import TelemetrySharer, CommandSharer, Command, Telemetry, RobotConfiguration
import threading
import numpy as np
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

        self._visualization = visualization

        self._telemetry_sharer = TelemetrySharer()
        self._command_sharer = CommandSharer()

        self._stop_event = threading.Event()
        self._visualization_thread = None
        self._simulation_thread = None

    def start(self):
        if self._visualization:
            self._visualization_thread = threading.Thread(
                target=visualization,
                daemon=True,
                args=(
                    self._stop_event,
                    self._telemetry_sharer,
                    self._robot_configuration,
                ))

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

    def add_command(self, command):
        self._command_sharer.add_command(command)

    def get_telemetry(self):
        self._telemetry_sharer.get_telemetry()

