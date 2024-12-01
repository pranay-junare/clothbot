import rospy
from std_msgs.msg import Float32MultiArray
import serial
import json
import time
import os

force_torque = [0.0]*6

def force_torque_callback(data):
    global force_torque
    force_torque = data.data

if __name__ == '__main__':
    dev = os.sys.argv[1]
    con = serial.Serial(
        dev,
        921600,
        timeout=10)

    messagePWM = {
        "ID": None,
        "PWM": [0.0]*6,
    }

    messageInfo = {
        "ID": None,
        "Info": "GET",
    }
    messageNo = 0

    # time.sleep(2)

    messageInfo["ID"] = messageNo
    messageNo += 1
    con.write(json.dumps(messageInfo).encode('utf-8')+b'\x00')
    data = con.read_until(b'\x00')[0:-1]
    data = json.loads(data.decode('utf-8'))
    # print(data)
    if "ERROR" in data:
        print("ERROR")
        print(data["ERROR"])
    if "NAME" not in data:
        print("ERROR")
        print("No NAME in data")
        exit()

    rospy.init_node(f'{data["NAME"].lower()}_haptic')
    sub = rospy.Subscriber(f"/{data['NAME'].lower()}_ft", Float32MultiArray, force_torque_callback)
    # sub = rospy.Subscriber("/lightning_ft", Float32MultiArray, force_torque_callback)
    print(f"Haptic {data['NAME']} subscribed to /{data['NAME'].lower()}_ft")
    

    while rospy.is_shutdown() == False:
        # Scale Force Torque sensor data
        # print(force_torque)
        # scaled_force_torque = [x*0.30*0.001 for x in force_torque]
        scaled_force_torque = [x*0.20 for x in force_torque]
        # print(scaled_force_torque)
        limited_scaled_force_torque = [min(x, 1.0) for x in scaled_force_torque]
        td, lr, fb = limited_scaled_force_torque[0:3]
    
        # bottom
        # top
        # left
        # right
        # back
        # front
        forces = [
            -fb if fb < 0 else 0,
            fb if fb > 0 else 0,
            -lr if lr < 0 else 0,
            lr if lr > 0 else 0,
            -td if td < 0 else 0,
            td if td > 0 else 0,
        ]
        # mask = [1, 1, 0, 0, 0, 0]
        # forces = [x*y for x, y in zip(forces, mask)]

        # print(forces)
        messagePWM["PWM"] = forces
        messagePWM["ID"] = messageNo
        messageNo += 1
        con.write(json.dumps(messagePWM).encode('utf-8')+b'\x00')
        data = con.read_until(b'\x00')[0:-1]
        data = json.loads(data.decode('utf-8'))
        # print(data)
        # print()

        rospy.sleep(0.005)

    con.close()