from pymavlink import mavutil
import time
from time import sleep
    
# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
    
# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

the_connection.mav.set_position_target_global_int_send(the_connection.target_system,the_connection.target_component,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,int(0b110111111000),int(-35.360916 * 10 ** 7),int(149.177985 * 10 ** 7),10,0,0,0,0,0,0,1.57,0.5)
ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print(f"Set Position Target ACK: {ack_msg}")