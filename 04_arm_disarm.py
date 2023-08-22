#Connect pymavlink with Pixhawk
from pymavlink import mavutil
import time

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

#Send manual fightmode
connection.mav.command_long_send(connection.target_system,connection.target_component,mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,
                                 mavutil.mavlink.MAV_MODE_MANUAL_DISARMED, #Set fightmode
                                 0,0,0,0,0,0) #Not use in this command

#Arm for 5 seconed
connection.arducopter_arm()
print("Arm")
time.sleep(5)

#Disarm
connection.arducopter_disarm()
print("Disarm")