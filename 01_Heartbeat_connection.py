#Connect pymavlink with Pixhawk
from pymavlink import mavutil

#set port and baud rate
baud = 57600
port = 'COM5'

#connection to Pixhawk using a serial port
connection = mavutil.mavlink_connection(port, baud=baud)
print(connection)

#Send message Heartbeat
connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GROUND_ROVER,mavutil.mavlink.MAV_AUTOPILOT_PX4, 
                              mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 65536, mavutil.mavlink.MAV_STATE_STANDBY)

#Wait Heartbeat message 
check_connection = connection.wait_heartbeat(timeout=5)
if check_connection is not None:
    print("Heartbeat connected!")
else:
    print(check_connection)