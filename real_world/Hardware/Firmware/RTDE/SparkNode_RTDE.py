import os
import json
import serial
import time
import pickle
import numpy as np
from Graph import LivePlot
import rospy
from std_msgs.msg import Float32MultiArray, Bool, Float32
import curses
import signal
import std_srvs.srv

# To run the RTDE stuff, use these three terminals:
# roscore
# SparkNode_RTDE.py
# run_RTDE.py

# Define a signal handler function
def signal_handler(sig, frame):
    print("Stopped")
    exit()

signal.signal(signal.SIGINT, signal_handler)

def get_distance(current, previous, max):
    dist = current - previous
    if(dist > max/2):
        dist -= max
    elif(dist < -max/2):
        dist += max   
    return dist/max*2*np.pi

jump = False
left_enable = False
right_enable = False
dat = ""
def safety_callback(msg):
    # robot_program_running : 0 = Stopped, 1 = Running
    global jump
    global left_enable
    global right_enable
    # Get the msg topic
    topic = msg._connection_header['topic'].split('/')[1]
    global dat
    dat += " "
    dat += topic
    if topic == 'left':
        left_enable = msg.mode == 1
    elif topic == 'right':
        right_enable = msg.mode == 1
    if left_enable or right_enable:
        jump = False

if __name__ == '__main__':
    con = serial.Serial(
        '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
        921600)
    print("Connection established, preparing to read data...")
    print("Press the Reset button on the Spark to start reading data")
    time.sleep(0.5) # Ignore bootloader messages
    # print(con.read_all()) # Clear buffer
    # con.read_until(b'\x00')[:-1] 
    con.reset_input_buffer()
    con.reset_output_buffer()
    con.read_until(b'\x00')[:-1] 

    print("Here we go!")
    # home_angle = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0, 0.0]
    #offsets_left = pickle.load(open("offsets_left.pickle", "rb"))
    #offsets_right = pickle.load(open("offsets_right.pickle", "rb"))
    #inverted = [-1, -1, 1, -1, -1, -1, 1] + [-1, -1, 1, -1, -1, -1, -1]
    inverted = [-1, -1, +1, -1, -1, -1, -1] 
    offsets_thunder = pickle.load(open("offsets_thunder.pickle", "rb"))
    num_angles = 7
    # plot = LivePlot(num_angles)
    max_raw_angle = 2**14
    previous_raw_angles = offsets_thunder
    # calculated_angles = home_angle
    calculated_angles = [0.0]*num_angles

    rospy.init_node('arm_state_publisher', anonymous=True)
    pose_publisher = rospy.Publisher('arm_state', Float32MultiArray, queue_size=1)
    enable_publisher = rospy.Publisher('enable_state', Bool, queue_size=1)
    # gripper_publisher = rospy.Publisher('gripper_state', Float32, queue_size=1)
    # rospy.Subscriber('/right/ur_hardware_interface/robot_program_running', ur_dashboard_msgs.msg.SafetyMode, safety_callback)
    # rospy.Subscriber('/left/ur_hardware_interface/robot_program_running', ur_dashboard_msgs.msg.SafetyMode, safety_callback)

    stdscr = curses.initscr()
    # curses.noecho()
    # curses.cbreak()

    stop_l = rospy.ServiceProxy('/left/ur_hardware_interface/dashboard/stop', std_srvs.srv.Trigger)
    stop_t = rospy.ServiceProxy('/right/ur_hardware_interface/dashboard/stop', std_srvs.srv.Trigger)

    init = False
    try:
        exit_flag = False
        while not exit_flag:
            data = con.read_until(b'\x00')[:-1]
            data = json.loads(data.decode('utf-8'))
            raw_angles = data['values']
            if(previous_raw_angles is not None):
                for i in range(num_angles):
                    # print(get_distance(raw_angles[i], previous_raw_angles[i], max_raw_angle))
                    if np.abs(get_distance(raw_angles[i], previous_raw_angles[i], max_raw_angle)) > 1 and init:
                        # stop_l()
                        # stop_t()
                        jump = True
                        # exit()
                    calculated_angles[i] += inverted[i] * get_distance(raw_angles[i], previous_raw_angles[i], max_raw_angle)
            init = True
            previous_raw_angles = raw_angles
            # plot.update(calculated_angles)
            pose_publisher.publish(Float32MultiArray(data=calculated_angles))
            enable_publisher.publish(Bool(data['enable_switch']))
            # gripper_publisher.publish(Float32(gripper_pos))

            #clear all lines:
            stdscr.clear()
            stdscr.addstr(0, 0, "Reading data...")
            stdscr.addstr(2, 0, "Status: ")
            stdscr.addstr(3, 4, str(data['status']))
            stdscr.addstr(6, 0, "Raw angles: ")
            stdscr.addstr(7, 4, str(raw_angles))
            if jump:
                stdscr.addstr(11, 0, "Angle jump detected, arms stopped!")
            else:
                stdscr.addstr(11, 0, "Arms Running...")
            # stdscr.addstr(13, 0, dat)
            stdscr.refresh()
    finally:
        os.system('reset')
        con.close()
        time.sleep(2)
        print("Connection closed")
