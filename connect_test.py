from pymavlink import mavutil

# Connect to SITL's MAVLink interface
the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')


print("Waiting for heartbeat from ArduCopter...")
the_connection.wait_heartbeat()
print(f" Heartbeat received from system {the_connection.target_system}, component {the_connection.target_component}")
