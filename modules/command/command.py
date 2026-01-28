"""
Decision-making logic.
"""

import math

from pymavlink import mavutil

from ..common.modules.logger import logger
from ..telemetry import telemetry


class Position:
    """
    3D vector struct.
    """

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class Command:  # pylint: disable=too-many-instance-attributes
    """
    Command class to make a decision based on recieved telemetry,
    and send out commands based upon the data.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> "tuple[True, Command] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a Command object.
        """
        return True, Command(cls.__private_key, connection, target, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"

        self.connection = connection
        self.target = target
        self.local_logger = local_logger

        # Thresholds
        # pylint: disable=invalid-name
        self.HEIGHT_TOLERANCE = 0.5  # meters
        self.ANGLE_TOLERANCE = math.radians(5)  # 5 degrees in radians

        # For average velocity calculation
        self.velocity_sum_x = 0.0
        self.velocity_sum_y = 0.0
        self.velocity_sum_z = 0.0
        self.data_count = 0

    def run(
        self, telemetry_data: telemetry.TelemetryData
    ) -> "tuple[True, str] | tuple[False, None]":
        """
        Make a decision based on received telemetry data.

        Returns (True, action_string) if a command was sent, (False, None) otherwise.
        """

        # Update average velocity tracking
        if telemetry_data.x_velocity is not None:
            self.velocity_sum_x += telemetry_data.x_velocity
        if telemetry_data.y_velocity is not None:
            self.velocity_sum_y += telemetry_data.y_velocity
        if telemetry_data.z_velocity is not None:
            self.velocity_sum_z += telemetry_data.z_velocity
        self.data_count += 1

        # Calculate and log average velocity
        avg_vx = self.velocity_sum_x / self.data_count
        avg_vy = self.velocity_sum_y / self.data_count
        avg_vz = self.velocity_sum_z / self.data_count
        self.local_logger.info(
            f"Average velocity: ({avg_vx:.2f}, {avg_vy:.2f}, {avg_vz:.2f}) m/s", True
        )

        # Check altitude
        if (
            telemetry_data.z is not None
            and abs(telemetry_data.z - self.target.z) > self.HEIGHT_TOLERANCE
        ):
            delta_z = self.target.z - telemetry_data.z

            # Send altitude change command
            self.connection.mav.command_long_send(
                1,  # target_system
                0,  # target_component
                mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,  # command (113)
                0,  # confirmation
                1.0,  # param1 (descent/climb rate in m/s), change from 0
                0,  # param2
                0,  # param3
                0,  # param4
                0,  # param5
                0,  # param6
                self.target.z,  # param7 (target altitude)
            )

            action = f"CHANGE ALTITUDE: {delta_z:.2f}"
            # self.local_logger.info(action, True)
            return True, action

        # Check yaw (orientation)
        if telemetry_data.yaw is not None:
            # Calculate required yaw to face target
            dx = self.target.x - telemetry_data.x
            dy = self.target.y - telemetry_data.y
            required_yaw = math.atan2(dy, dx)

            # Calculate angle difference (handling wraparound)
            angle_diff = required_yaw - telemetry_data.yaw

            # Normalize to [-π, π]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            if abs(angle_diff) > self.ANGLE_TOLERANCE:
                # Convert to degrees for command
                angle_diff_deg = math.degrees(angle_diff)
                direction = -1 if angle_diff_deg >= 0 else 1  # 1=clockwise, -1=counter-clockwise

                # Send yaw change command (relative)
                self.connection.mav.command_long_send(
                    1,  # target_system
                    0,  # target_component
                    mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command (115)
                    0,  # confirmation
                    angle_diff_deg,  # param1 (target angle in degrees)
                    5.0,  # param2 (angular speed in deg/s) - CHANGE FROM 0
                    direction,  # param3 (direction: 1=clockwise, -1=counter-clockwise, not used for relative)
                    1,  # param4 (relative=1, absolute=0)
                    0,  # param5
                    0,  # param6
                    0,  # param7
                )

                action = f"CHANGE YAW: {angle_diff_deg:.2f}"
                # self.local_logger.info(action, True)
                return True, action

        # No action needed
        return False, None


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
