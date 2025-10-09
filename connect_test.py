from pymavlink import mavutil


the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

print("Waiting for heartbeat...")
the_connection.wait_heartbeat()
print(f"Heartbeat received from system {the_connection.target_system}, component {the_connection.target_component}")


with open("telemetry_log.txt", "w") as f:
    while True:
        msg = the_connection.recv_match(blocking=True) #this allows us to receive all the messages
        if msg:
            f.write(str(msg) + "\n") #we are writing the messages to a file
            f.flush()

"""
MAVLink is a lightweight messaging protocol designed for communication with drones and robotic vehicles.
Using the pymavlink library, we send an initial heartbeat message to activate the device. By default, the flight controller doesn't stream telemetry data until it detects an active connection.
We send a dummy heartbeat from our computer to the device, which acts as a "wake-up call." This triggers the device to begin transmitting its actual telemetry data with real parameter values.


"""
