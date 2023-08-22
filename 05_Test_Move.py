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
                                 mavutil.mavlink.MAV_MODE_MANUAL_DISARMED,
                                 0,0,0,0,0,0)

time.sleep(1)

connection.arducopter_arm()
print("Armed")

# Send manual control command
start_time = time.time()
duration = 3  # in seconds
while time.time() - start_time < duration:
    connection.mav.manual_control_send( 
        connection.target_system,
        0, # x 
        0, 1000, # y,z --> change move here y = left/right , z = front/back
        0, # r
        0  # button
    )
    time.sleep(0.1)  # Send control commands at 10 Hz

print("Manual control sent move servo for 3 seconds.")

connection.mav.manual_control_send(
    connection.target_system,
    0, 0, 0, 0, # x, y, z, r
    0 # button
    )
time.sleep(0.1)  # Send control commands at 10 Hz


connection.arducopter_disarm()
print("Disarmed")