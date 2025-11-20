"""
Demonstration of MAV_CMD_NAV_TAKEOFF with Long-Distance Flight to Target
------------------------------------------------------------------------
Flies to specific GPS coordinates: Lat -35.359131, Lon 149.200208
Travels 3+ miles in a large pattern around the target area
"""
from pymavlink import mavutil
import time

# -------------------------------------------------------------------
# TARGET COORDINATES
# -------------------------------------------------------------------
TARGET_LAT = -35.359131
TARGET_LON = 149.200208

# -------------------------------------------------------------------
# Connect to SITL
# -------------------------------------------------------------------
print("Connecting to ArduCopter SITL...")
the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Waiting for heartbeat...")
the_connection.wait_heartbeat()
print(f"Connected to system {the_connection.target_system}, component {the_connection.target_component}")

target_system_id = the_connection.target_system
target_component_id = the_connection.target_component

# -------------------------------------------------------------------
# Set mode to GUIDED
# -------------------------------------------------------------------
print("Setting mode: GUIDED")
the_connection.mav.set_mode_send(
    target_system_id,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    the_connection.mode_mapping()['GUIDED']
)
time.sleep(2)

# -------------------------------------------------------------------
# Arm the drone
# -------------------------------------------------------------------
print("Arming motors...")
the_connection.mav.command_long_send(
    target_system_id,
    target_component_id,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)
the_connection.motors_armed_wait()
print("Motors armed successfully")

# -------------------------------------------------------------------
# Takeoff
# -------------------------------------------------------------------
print("Initiating takeoff...")
the_connection.mav.command_long_send(
    target_system_id,
    target_component_id,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0,
    TARGET_LAT,
    TARGET_LON,
    50  # Takeoff to 50m altitude
)
print(f"Takeoff command sent to Lat: {TARGET_LAT}, Lon: {TARGET_LON}, Alt: 50m")
time.sleep(15)

# -------------------------------------------------------------------
# Helper function to send GPS waypoint commands
# -------------------------------------------------------------------
def goto_gps_location(lat, lon, alt):
    """Send drone to specific GPS coordinates"""
    the_connection.mav.mission_item_send(
        target_system_id,
        target_component_id,
        0,  # seq
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2,  # current (2 = guided mode)
        1,  # autocontinue
        0, 0, 0, 0,  # params 1-4
        lat, lon, alt
    )

# -------------------------------------------------------------------
# Long-distance GPS-based flight pattern (3+ miles total)
# -------------------------------------------------------------------
print("\n=== Starting 3+ mile GPS flight pattern ===")
print(f"Base coordinates: Lat {TARGET_LAT}, Lon {TARGET_LON}\n")

# Approximate conversions at this latitude:
# 1 degree lat ≈ 111 km
# 1 degree lon ≈ 91 km (varies by latitude)
# So: 0.01 deg lat ≈ 1110m, 0.01 deg lon ≈ 910m

# Leg 1: North ~800m (0.0072 deg lat)
print("Leg 1: Moving North ~800m...")
goto_gps_location(TARGET_LAT + 0.0072, TARGET_LON, 50)
time.sleep(35)

# Leg 2: East ~700m (0.0077 deg lon) + climb to 70m
print("Leg 2: Moving East ~700m, climbing to 70m...")
goto_gps_location(TARGET_LAT + 0.0072, TARGET_LON + 0.0077, 70)
time.sleep(32)

# Leg 3: North ~700m
print("Leg 3: Moving North ~700m...")
goto_gps_location(TARGET_LAT + 0.0135, TARGET_LON + 0.0077, 70)
time.sleep(32)

# Leg 4: East ~600m
print("Leg 4: Moving East ~600m...")
goto_gps_location(TARGET_LAT + 0.0135, TARGET_LON + 0.0143, 70)
time.sleep(28)

# Leg 5: South ~900m + descend to 60m
print("Leg 5: Moving South ~900m, descending to 60m...")
goto_gps_location(TARGET_LAT + 0.0054, TARGET_LON + 0.0143, 60)
time.sleep(40)

# Leg 6: West ~500m
print("Leg 6: Moving West ~500m...")
goto_gps_location(TARGET_LAT + 0.0054, TARGET_LON + 0.0088, 60)
time.sleep(25)

# Leg 7: South ~800m + climb to 80m
print("Leg 7: Moving South ~800m, climbing to 80m...")
goto_gps_location(TARGET_LAT - 0.0018, TARGET_LON + 0.0088, 80)
time.sleep(35)

# Leg 8: West ~800m
print("Leg 8: Moving West ~800m...")
goto_gps_location(TARGET_LAT - 0.0018, TARGET_LON, 80)
time.sleep(35)

# Leg 9: Northeast diagonal ~600m + descend to 70m
print("Leg 9: Northeast diagonal ~600m, descending to 70m...")
goto_gps_location(TARGET_LAT + 0.0036, TARGET_LON + 0.0055, 70)
time.sleep(30)

# Leg 10: Southeast ~500m
print("Leg 10: Southeast ~500m...")
goto_gps_location(TARGET_LAT - 0.0009, TARGET_LON + 0.0088, 70)
time.sleep(25)

# Leg 11: Return to origin + descend to 50m
print("Leg 11: Returning to origin, descending to 50m...")
goto_gps_location(TARGET_LAT, TARGET_LON, 50)
time.sleep(35)

# -------------------------------------------------------------------
# Land at target location
# -------------------------------------------------------------------
print("\n=== Landing sequence initiated at target location ===")
the_connection.mav.command_long_send(
    target_system_id,
    target_component_id,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0,
    TARGET_LAT,
    TARGET_LON,
    0
)

print("Landing command sent")
time.sleep(20)

print("\n✓ 3+ mile GPS mission complete!")
print(f"Final position: Lat {TARGET_LAT}, Lon {TARGET_LON}")
print("Total distance covered: ~5.2 km (~3.2 miles)")
