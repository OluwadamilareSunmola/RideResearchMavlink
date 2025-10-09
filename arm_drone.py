"""

ARM COMMAND

Arming a drone is the process of enabling a drone to begin operation like starting to spin the motors

We are using the command long protocol

When we send a command, the gcs (Ground station control) or program sends the message command long
to the drone, and when drone receives that message, it tries to act upon the command
if that is successful, it sends back an acknowledgement, program can tell if command succedded or failed


Target System identifies which system (vehicle) you’re addressing: this can be Ground control station, vehicle, or raspberry pi
Every target system has multiple components eg a drone has camera and so on

We can be precise on where we are sending commands to. Example is camera, flight controller and so on

Command-Long parameters:

    target_system: Target system you are sending command to
    target_component: Target component you are sending command to, which component should executre the command
    command: enumeration of command you are sending, this is the command id, example can be takeoff, land local and many others
    confirmation: 0 for confirmation for first attempt, 1-255 if resending
    param1 - param7 this is a command-specific parameter


    example mav_cmd_component_arm_disarm (400) enables or disables the motors

    Param1: takes a number 0 or 1, disarm or arm
    Param2: this is 0 arm disarm unless prevented by safety checks or force arm/disarm


    Command_ack field parameters:
        command: 400 if command went true
        result of the command -> 0 means command is valud and was executed
        progress: used for commands that take time, usually always 0
        

        COMMAND_ACK {command : 400, result : 0, progress : 0, result_param2 : 0, target_system : 255, target_component : 0}
"""


from pymavlink import mavutil
import time

the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

the_connection.wait_heartbeat()

target_system_id = the_connection.target_system
target_component_id = the_connection.target_component



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

msg = the_connection.recv_match(type="COMMAND_ACK", blocking = True) #we want to get acknowledgment message
print(msg)
print('Command is sent')
