
"""

Mavlink movement the commands avaialable are set_position_target_load_ned using local reference frame


There is also set_attitude_target which is used when trying to implement controller at attitude level


Purpose:
Used to control the drone’s position, velocity, acceleration, and yaw in a local NED (North-East-Down) coordinate frame.
Typically used in OFFBOARD or GUIDED mode to send position/yaw targets.

Parameters:
1. time_boot_ms:
   - Time since system boot in milliseconds.
   - Used for message synchronization.

2. target_system:
   - System ID of the drone being controlled (e.g., 1).

3. target_component:
   - Component ID, usually 1 for the autopilot (MAV_COMP_ID_AUTOPILOT1).

4. coordinate_frame:
   - Frame of reference for the position data.
   - Usually MAV_FRAME_LOCAL_NED (North-East-Down) -> ekf first position where you get position estimate
   - X = North (+forward), Y = East (+right), Z = Down (+downward).

5. type_mask:
   - Bitmask specifying which fields to ignore.
   - Allows control over only position, velocity, or acceleration.
   - Example: 0b0000111111000111 = ignore velocity, acceleration, and yaw rate.

6. x, y, z:
   - Target position (in meters) relative to the local NED origin.
   - Example: x=10 → move 10 m north, y=0 → no east/west change, z=-5 → 5 m above origin.

7. yaw:
   - Target heading (in radians).
   - 0 rad = North, π/2 = East, π = South, 3π/2 = West.

8. yaw_rate:
   - Desired yaw rotation rate (radians/sec).
   - Used when yaw is ignored.

Coordinate Reference:
  ↑ North (x)
  |
  |
  o-----> East (y)
  ↓ Down (z)

Example (PyMAVLink):
the_connection.mav.set_position_target_local_ned_send(
    time_boot_ms=int(time.time() * 1000),
    target_system=1,
    target_component=1,
    coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    type_mask=0b0000111111000111,
    x=5.0, y=0.0, z=-2.0,
    vx=0, vy=0, vz=0,
    afx=0, afy=0, afz=0,
    yaw=0, yaw_rate=0
A bitmask that says what to ignore (position, velocity, yaw, etc.).

It decides what kind of control you’re sending:

Only position → ignore velocity, acceleration, yaw rate

Only velocity → ignore position, acceleration

Example: 0b0000111111000111 means “control position only”.

🧩 What is type_mask?

type_mask is basically a switchboard.

It tells the autopilot:

“Ignore these fields — only pay attention to the rest.”

When you send a MAVLink message like:

set_position_target_local_ned_send(...)


you can include up to 13 parameters (position, velocity, acceleration, yaw, etc.).
But you don’t always want to control everything — sometimes you just want to send a position target, or just a velocity target.

That’s where type_mask comes in.

How it works

Each bit in the 16-bit type_mask corresponds to one thing you can tell the autopilot to ignore.

If a bit = 1 → that field is ignored
If a bit = 0 → that field is used

Example layout (simplified):
Bit	Field Ignored	Meaning
0	x	Ignore X position
1	y	Ignore Y position
2	z	Ignore Z position
3	vx	Ignore X velocity
4	vy	Ignore Y velocity
5	vz	Ignore Z velocity
6	ax	Ignore X acceleration
7	ay	Ignore Y acceleration
8	az	Ignore Z acceleration
10	yaw	Ignore yaw angle - yaw target is used when you want drone to face a certain direction
11	yaw_rate	Ignore yaw rate - yaw rate is used when you want drone to constantly turn in a certain direction

There are more bits, but these are the most common.

MAVLINK POSITION FRAMES EXPLANATION
-----------------------------------

Two main movement command types:
1. set_position_target_local_ned  --> LOCAL reference frame
2. set_position_target_global_int --> GLOBAL reference frame


===================================
LOCAL NED FRAME
===================================

Purpose:
Used to control position/velocity/acceleration in a small, local 3D space.
Coordinates are in meters relative to the drone’s EKF origin 
(the first point where it got a good GPS fix or IMU position estimate).

Reference axes (NED):
  X -> North  (+forward)
  Y -> East   (+right)
  Z -> Down   (+downward)
  (Negative Z means up)

Example:
x = 10  → move 10m north
y = 0   → no east/west change
z = -5  → go 5m up (because NED down is positive)

Use cases:
- Short-range movement
- Precise hovering or takeoff
- Indoor or GPS-denied flight
- Path following using relative distances

Example command:
the_connection.mav.set_position_target_local_ned_send(
    time_boot_ms,
    target_system,
    target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111000111,  # ignore velocity/accel/yaw rate
    10, 0, -5,            # x, y, z in meters (relative)
    0, 0, 0,              # velocity (ignored)
    0, 0, 0,              # acceleration (ignored)
    0, 0)                 # yaw and yaw rate


===================================
GLOBAL FRAME
===================================

Purpose:
Used to move the drone to an absolute GPS coordinate on Earth.
Coordinates are in latitude, longitude, and altitude.

Reference system:
- Based on WGS-84 Earth model (real GPS positions)
- Altitude is usually relative to takeoff point (AGL)

Frame types:
MAV_FRAME_GLOBAL               -> Altitude above sea level (AMSL)
MAV_FRAME_GLOBAL_RELATIVE_ALT  -> Altitude relative to takeoff
MAV_FRAME_GLOBAL_TERRAIN_ALT   -> Altitude above terrain

Example:
target_lat = int(-35.3629849 * 1e7)  # latit*_

"""
from pymavlink import mavutil
import time

# Connect to drone
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
connection.wait_heartbeat()
print(f"Connected to system {connection.target_system}")

def set_mode(mode_id):
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id, 0, 0, 0, 0, 0)
    connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Mode set to {mode_id}")


def goto_position(lat, lon, alt, speed_x=0, speed_y=0, duration=100):
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    
    for i in range(duration):
        connection.mav.set_position_target_global_int_send(
            0,  # time_boot_ms
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111000111 if (speed_x or speed_y) else 0b0000111111111000,
            lat_int, lon_int, alt,
            speed_x, speed_y, 0,  # velocities
            0, 0, 0,  # accelerations
            0, 0)  # yaw, yaw_rate
        
        time.sleep(0.5)
        if (i + 1) % 20 == 0:
            print(f"Command sent {i+1}/{duration}")

# Set to GUIDED mode
set_mode(4)
time.sleep(1)

# Go to target position
goto_position(
    lat=-35.3629849 + 0.045,  # ~5km north
    lon=149.1649185 + 0.054,  # ~5km east
    alt=100,
    speed_x=25,  # 25 m/s north
    speed_y=25,  # 25 m/s east
    duration=500
)

# Monitor position
while True:
    msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    print(msg)
