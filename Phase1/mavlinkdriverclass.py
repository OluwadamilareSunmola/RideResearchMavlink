"""
===========================================================

This script demonstrates how to use pymavlink to integrate
a custom Time-of-Flight (ToF) sensor (e.g., DWM3000) with
a drone or SITL environment through MAVLink messages.
It includes:
    - MAVLink connection setup
    - Sending commands (e.g., set flight mode)
    - Sending and receiving ToF data
    - Retrieving telemetry (GPS, position, heartbeat)
===========================================================
"""

from pymavlink import mavutil
import time
from collections import deque


class DWM_MAVLink_Driver:
    """
    MAVLink driver for DWM3000 integration.
    Handles communication between drone and sensor systems using MAVLink.
    """

    def __init__(self, connection_string='udp:127.0.0.1:14550'):
        """
        Initialize MAVLink driver and wait for connection.

        Args:
            connection_string (str): MAVLink endpoint (UDP, serial, etc.)
        """
        # Establish MAVLink connection (e.g., with SITL or QGroundControl)
        self.connection = mavutil.mavlink_connection(connection_string)

        # Wait until a heartbeat is received from the target system
        # This ensures that communication is active and synchronized
        self.connection.wait_heartbeat()
        print(f"Connected to system {self.connection.target_system}")

        # Maintain a queue of received ToF (distance) messages
        self.tof_queue = deque(maxlen=100)
        self.last_tof_time = 0

    # ----------------------------------------------------------------------
    # ToF Data Handling
    # ----------------------------------------------------------------------

    def send_tof_data(self, distance_cm, device_id=1):
        """
        Send Time-of-Flight (ToF) data using the MAVLink DISTANCE_SENSOR message.

        Args:
            distance_cm (float): Measured distance in centimeters
            device_id (int): Unique ID for this ToF sensor
        """
        # MAVLink timestamp in milliseconds (wraps at 2^32)
        time_boot_ms = int(time.time() * 1000) % (2**32)

        # Send DISTANCE_SENSOR message through MAVLink
        self.connection.mav.distance_sensor_send(
            time_boot_ms,                                 # time since boot
            10,                                           # min_distance (cm)
            1000,                                         # max_distance (cm)
            distance_cm,                                  # current_distance (cm)
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,    # sensor type
            device_id,                                    # device ID
            mavutil.mavlink.MAV_SENSOR_ROTATION_NONE,     # orientation
            0                                             # covariance
        )
        print(f"ToF data sent: distance={distance_cm}cm, device={device_id}")

    def receive_tof_data(self, timeout=1):
        """
        Receive incoming ToF data from DISTANCE_SENSOR messages.

        Args:
            timeout (float): Timeout in seconds for waiting on a message.

        Returns:
            dict or None: Parsed ToF data if received.
        """
        msg = self.connection.recv_match(
            type='DISTANCE_SENSOR', blocking=True, timeout=timeout
        )

        if msg:
            # Store message in queue for historical access
            self.tof_queue.append(msg)
            self.last_tof_time = time.time()

            # Return structured dictionary of key fields
            return {
                'time_boot_ms': msg.time_boot_ms,
                'distance_cm': msg.current_distance,
                'device_id': msg.id,
                'min_distance': msg.min_distance,
                'max_distance': msg.max_distance
            }
        return None

    # ----------------------------------------------------------------------
    # Command and Acknowledgment
    # ----------------------------------------------------------------------

    def send_command(self, command, param1=0, param2=0, param3=0,
                     param4=0, param5=0, param6=0, param7=0):
        """
        Send a MAVLink command (COMMAND_LONG).

        Args:
            command (int): MAVLink command ID (e.g., MAV_CMD_NAV_TAKEOFF)
            param1–param7: Command-specific parameters.
        """
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            command,
            0,  # confirmation
            param1, param2, param3, param4, param5, param6, param7
        )
        print(f"Command {command} sent")

    def receive_ack(self, timeout=3):
        """
        Wait for COMMAND_ACK response confirming command success/failure.

        Args:
            timeout (float): Seconds to wait for acknowledgment.

        Returns:
            bool: True if ACK indicates success, False otherwise.
        """
        msg = self.connection.recv_match(
            type='COMMAND_ACK', blocking=True, timeout=timeout
        )
        if msg:
            print(f"ACK received: command={msg.command}, result={msg.result}")
            return msg.result == 0  # MAV_RESULT_ACCEPTED = 0
        return False

    # ----------------------------------------------------------------------
    # Telemetry
    # ----------------------------------------------------------------------

    def get_telemetry(self):
        """
        Collect flight telemetry such as local position, GPS, and arm state.

        Returns:
            dict: Telemetry dictionary with available data.
        """
        telemetry = {}

        # Retrieve local NED position (x, y, z)
        msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            telemetry['position'] = {'x': msg.x, 'y': msg.y, 'z': msg.z}

        # Retrieve global GPS position
        msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            telemetry['gps'] = {
                'lat': msg.lat / 1e7,
                'lon': msg.lon / 1e7,
                'alt': msg.relative_alt / 1000.0
            }

        # Check heartbeat to determine armed state
        msg = self.connection.recv_match(type='HEARTBEAT', blocking=False)
        if msg:
            telemetry['armed'] = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )

        return telemetry

    # ----------------------------------------------------------------------
    # Queue Utilities
    # ----------------------------------------------------------------------

    def get_tof_queue_size(self):
        """Return number of stored ToF messages."""
        return len(self.tof_queue)

    def clear_tof_queue(self):
        """Clear all ToF messages in queue."""
        self.tof_queue.clear()


# ===============================================================
# TEST SEQUENCE
# ===============================================================

# Step 1: Establish MAVLink connection
driver = DWM_MAVLink_Driver('udp:127.0.0.1:14550')

# ---------------------------------------------------------------
# TEST 1: Send a MAVLink command (Set flight mode to GUIDED)
# ---------------------------------------------------------------
print("\n=== Test 1: Send Command ===")
driver.send_command(
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,                # Command: change mode
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # Enable custom mode flag
    4                                                   # Mode ID 4 = GUIDED mode
)

# Wait for acknowledgment
if driver.receive_ack():
    print("Command acknowledged")

time.sleep(1)

# ---------------------------------------------------------------
# TEST 2: Send simulated ToF data (DISTANCE_SENSOR messages)
# ---------------------------------------------------------------
print("\n=== Test 2: Send ToF Data ===")
for i in range(5):
    # Generate fake distance readings (e.g., 100, 110, 120, 130, 140 cm)
    distance = 100 + i * 10
    driver.send_tof_data(distance_cm=distance, device_id=1)
    time.sleep(0.5)

# ---------------------------------------------------------------
# TEST 3: Receive telemetry and ToF data
# ---------------------------------------------------------------
print("\n=== Test 3: Receive Telemetry ===")
for i in range(10):
    telemetry = driver.get_telemetry()
    if telemetry:
        print(f"Telemetry: {telemetry}")

    # Check if ToF messages are being received
    tof_data = driver.receive_tof_data(timeout=0.1)
    if tof_data:
        print(f"ToF received: {tof_data}")

    time.sleep(0.5)

# ---------------------------------------------------------------
# FINAL SUMMARY
# ---------------------------------------------------------------
print(f"\nPhase 1 Complete! ToF messages in queue: {driver.get_tof_queue_size()}")
