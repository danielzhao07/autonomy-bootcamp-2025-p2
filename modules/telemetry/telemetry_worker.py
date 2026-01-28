"""
Telemtry worker that gathers GPS data.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import telemetry
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def telemetry_worker(
    period: float,
    connection: mavutil.mavfile,
    telemetry_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process. Gathers telemetry data from the drone.

    period: Timeout period for receiving messages
    connection: MAVLink connection to the drone
    telemetry_queue: Queue to send TelemetryData to Command worker
    controller: Worker controller for managing worker state
    """
    # =============================================================================================
    #                          ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    # Instantiate logger
    worker_name = pathlib.Path(__file__).stem
    process_id = os.getpid()
    result, local_logger = logger.Logger.create(f"{worker_name}_{process_id}", True)
    if not result:
        print("ERROR: Worker failed to create logger")
        return

    # Get Pylance to stop complaining
    assert local_logger is not None

    local_logger.info("Logger initialized", True)

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Instantiate class object (telemetry.Telemetry)

    result, telem = telemetry.Telemetry.create(period, connection, local_logger)
    if not result:
        local_logger.error("Failed to create Telemetry", True)
        return

    assert telem is not None

    local_logger.info("Telemetry created", True)

    # Main loop: do work.
    while not controller.is_exit_requested():
        controller.check_pause()
        result, telemetry_data = telem.run()

        if result:
            # Successfully got telemetry data, send to queue
            telemetry_queue.queue.put(telemetry_data)
            local_logger.info(f"Sent telemetry data: {telemetry_data}", True)
        else:
            # Timeout occurred, restart and try again
            local_logger.warning("Telemetry timeout, restarting", True)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
