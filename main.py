from simulation import simulation
from visualization import visualization
from control_client import control_client
from tools import TelemetrySharer, CommandSharer
import threading

telemetry_sharer = TelemetrySharer()
command_sharer = CommandSharer()

visualization_thread = threading.Thread(target=visualization,
                                        daemon=True,
                                        args=(telemetry_sharer, ))


simulation_thread = threading.Thread(target=simulation,
                                     daemon=True,
                                     args=(
                                         telemetry_sharer,
                                         command_sharer,
                                     ))


control_client_thread = threading.Thread(target=control_client,
                                         daemon=True,
                                         args=(
                                             telemetry_sharer,
                                             command_sharer,
                                         ))

visualization_thread.start()
simulation_thread.start()
control_client_thread.start()

visualization_thread.join()
simulation_thread.join()
control_client_thread.join()
