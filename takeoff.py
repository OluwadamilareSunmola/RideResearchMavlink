
"""


MAV_CMD_NAV_TAKEOFF allows take off from the ground

MAV_CMD_NAV_TAKEOFF parameters:
    param1: Minimimum pitch - this is angle vehicle should maintain during climb
    param2, 3 is 0
    param4: yaw angle: dsesired heading at takeoff whether it is north south west east
    param5: latitude
    param6: longitude
    param7: altitude



    Make sure to put in guided mode
"""
from pymavlink import mavutil
import time

the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

the_connection.wait_heartbeat()

target_system_id = the_connection.target_system
target_component_id = the_connection.target_component



time.sleep(2)

#arm the drone before takeoff

the_connection.mav.command_long_send(
    target_system_id,
    target_component_id,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,
    0,
    0,
    0,
    0,
    0,
    0
)

the_connection.mav.command_long_send(
    target_system_id,
    target_component_id,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0,          # param1: minimum pitch (0° for copters)
    0, 0,       # param2, param3: unused
    90,         # param4: yaw angle (90° = East)
    37.4276,    # param5: latitude
    -122.1697,  # param6: longitude
    10          # param7: target altitude (10 m)
)

"""
the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4,  # 4 = GUIDED mode
    0, 0, 0, 0, 0)

"""

while True:
    msg = the_connection.recv_match(
        type=["COMMAND_ACK", "ALTITUDE", "HEARTBEAT"],
        blocking=True
    )
    if msg:
        data = msg.to_dict()
        print(data)
print('Command is sent')
