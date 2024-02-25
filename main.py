from simulation import simulation
from visualization import visualization
from control_client import control_client
from tools import TelemetrySharer, CommandSharer
import threading
from time import sleep


def main():
    telemetry_sharer = TelemetrySharer()
    command_sharer = CommandSharer()

    stop_event = threading.Event()

    visualization_thread = threading.Thread(target=visualization,
                                            daemon=True,
                                            args=(
                                                stop_event,
                                                telemetry_sharer,
                                            ))

    simulation_thread = threading.Thread(target=simulation,
                                         daemon=True,
                                         args=(
                                             stop_event,
                                             telemetry_sharer,
                                             command_sharer,
                                         ))

    control_client_thread = threading.Thread(target=control_client,
                                             daemon=True,
                                             args=(
                                                 stop_event,
                                                 telemetry_sharer,
                                                 command_sharer,
                                             ))

    visualization_thread.start()
    simulation_thread.start()
    control_client_thread.start()

    visualization_thread.join()
    simulation_thread.join()
    control_client_thread.join()


if __name__ == "__main__":
    main()
