
from pymavlink import mavutil
import time

class MAVLinkConnection:
    def __init__(self, connection_string='udp:127.0.0.1:14550'):
        self.connection = mavutil.mavlink_connection(connection_string)
        self.connection.wait_heartbeat()
        print(f"Heartbeat from system {self.connection.target_system}, component {self.connection.target_component}")
        
    def get_connection(self):
        return self.connection

# Usage
mav_conn = MAVLinkConnection('udp:127.0.0.1:14550')
connection = mav_conn.get_connection()
