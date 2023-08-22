import tkinter as tk
from pymavlink import mavutil
import time

baud = 57600
port = 'COM5'

# Establish a connection to QGroundControl using a serial port
connection = mavutil.mavlink_connection(port, baud=baud)

connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GROUND_ROVER, mavutil.mavlink.MAV_AUTOPILOT_PX4,
                              mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 65536, mavutil.mavlink.MAV_STATE_STANDBY)
# Wait for a heartbeat message
msg = connection.wait_heartbeat(timeout=5)
if msg is not None:
    print("Heartbeat connected!")
    print(msg)
else:
    print("No heartbeat received.")

connection.mav.command_long_send(connection.target_system,connection.target_component,mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,
                                 mavutil.mavlink.MAV_MODE_MANUAL_DISARMED,
                                 0,0,0,0,0,0)

root = tk.Tk()
root.title("Chop")

class MoveFront:
    def __init__(self,root):
        self.root = root
        self.is_pressed = False
        self.button = tk.Button(root, text="Front",command=self.toggle_press)
        self.button.pack()
        self.update_button()

    def toggle_press(self):
        connection.arducopter_arm()
        self.is_pressed = not self.is_pressed
        self.update_button()

    def update_button(self):
        if self.is_pressed:
            self.perform_action()
            self.root.after(100, self.update_button)  #Call update_button every 100 milliseconds
        else:
            self.button.config(relief=tk.RAISED)

    def perform_action(self):
        connection.mav.manual_control_send(
            connection.target_system,
            0, 0, 1000, 0,
            0)
        time.sleep(0.1)
        self.button.config(relief=tk.SUNKEN)

class MoveBack:
    def __init__(self,root):
        self.root = root
        self.is_pressed = False
        self.button = tk.Button(root, text="Back",command=self.toggle_press)
        self.button.pack()
        self.update_button()

    def toggle_press(self):
        connection.arducopter_arm()
        self.is_pressed = not self.is_pressed
        self.update_button()

    def update_button(self):
        if self.is_pressed:
            self.perform_action()
            self.root.after(100, self.update_button)  #Call update_button every 100 milliseconds
        else:
            self.button.config(relief=tk.RAISED)

    def perform_action(self):
        connection.mav.manual_control_send(
            connection.target_system,
            0, 0, -1000, 0,
            0)
        time.sleep(0.1)
        self.button.config(relief=tk.SUNKEN)

class MoveLeft:
    def __init__(self,root):
        self.root = root
        self.is_pressed = False
        self.button = tk.Button(root, text="Left",command=self.toggle_press)
        self.button.pack()
        self.update_button()

    def toggle_press(self):
        connection.arducopter_arm()
        self.is_pressed = not self.is_pressed
        self.update_button()

    def update_button(self):
        if self.is_pressed:
            self.perform_action()
            self.root.after(100, self.update_button)  #Call update_button every 100 milliseconds
        else:
            self.button.config(relief=tk.RAISED)

    def perform_action(self):
        connection.mav.manual_control_send(
            connection.target_system,
            0, -1000, 0, 0,
            0)
        time.sleep(0.1)
        self.button.config(relief=tk.SUNKEN)

class MoveRight:
    def __init__(self,root):
        self.root = root
        self.is_pressed = False
        self.button = tk.Button(root, text="Right",command=self.toggle_press)
        self.button.pack()
        self.update_button()

    def toggle_press(self):
        connection.arducopter_arm()
        self.is_pressed = not self.is_pressed
        self.update_button()

    def update_button(self):
        if self.is_pressed:
            self.perform_action()
            self.root.after(100, self.update_button)  #Call update_button every 100 milliseconds
        else:
            self.button.config(relief=tk.RAISED)

    def perform_action(self):
        connection.mav.manual_control_send(
            connection.target_system,
            0, 1000, 0, 0,
            0)
        time.sleep(0.1)
        self.button.config(relief=tk.SUNKEN)

def Arm():
    connection.arducopter_arm()
    print("Armed")

def Disarm():
    connection.arducopter_disarm()
    print("Disarmed")


Armed = tk.Button(root, text="Arm", command=Arm)
Armed.pack()

MoveFront(root)
MoveBack(root)
MoveLeft(root)
MoveRight(root)

Disarmed = tk.Button(root, text="Disarm", command=Disarm)
Disarmed.pack()

root.mainloop()

#Disarm
connection.arducopter_disarm()
print("Disarmed")