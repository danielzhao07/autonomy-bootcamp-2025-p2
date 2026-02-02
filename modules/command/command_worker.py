"""
Command worker to make decisions based on Telemetry Data.
"""

import os
import pathlib
import time

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import command
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def command_worker(
    target: command.Position,
    height_tolerance: float,
    angle_tolerance: float,
    connection: mavutil.mavfile,
    telemetry_queue: queue_proxy_wrapper.QueueProxyWrapper,
    report_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process. Makes decisions based on telemetry data.

    target: Target position to maintain
    height_tolerance: Tolerance for altitude adjustments (meters)
    angle_tolerance: Tolerance for yaw adjustments (degrees)
    connection: MAVLink connection to the drone
    telemetry_queue: Input queue receiving TelemetryData
    report_queue: Output queue for action strings
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
    # Instantiate class object (command.Command)
    result, cmd = command.Command.create(
        connection, target, height_tolerance, angle_tolerance, local_logger
    )
    if not result:
        local_logger.error("Failed to create Command", True)
        return

    assert cmd is not None

    local_logger.info("Command created", True)

    # Track cumulative velocities for average calculation
    total_x_velocity = 0.0
    total_y_velocity = 0.0
    total_z_velocity = 0.0
    data_count = 0

    # Main loop: do work.
    while not controller.is_exit_requested():
        controller.check_pause()
        if not telemetry_queue.queue.empty():
            telemetry_data = telemetry_queue.queue.get()
            # local_logger.info(f"Received telemetry: {telemetry_data}", True)

            if telemetry_data is None:
                continue

            # Update cumulative velocities and count
            if telemetry_data.x_velocity is not None:
                total_x_velocity += telemetry_data.x_velocity
            if telemetry_data.y_velocity is not None:
                total_y_velocity += telemetry_data.y_velocity
            if telemetry_data.z_velocity is not None:
                total_z_velocity += telemetry_data.z_velocity
            data_count += 1

            # Calculate and log average velocity vector
            avg_x = total_x_velocity / data_count
            avg_y = total_y_velocity / data_count
            avg_z = total_z_velocity / data_count
            local_logger.info(
                f"Average Velocity - x: {avg_x}, y: {avg_y}, z: {avg_z}",
                True,
            )

            result, action = cmd.run(telemetry_data)

            if result:
                # Send action string to report queue
                report_queue.queue.put(action)
        else:
            time.sleep(0.01)  # Small sleep when queue is empty


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
