import os
import json
import serial
import time
import pickle
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Bool, Float32
import signal

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

if __name__ == '__main__':
    dev = os.sys.argv[1]
    def init_connection():
        con = serial.Serial(dev, 921600)
        time.sleep(0.5) # Ignore bootloader messages
        con.reset_input_buffer()
        con.reset_output_buffer()
        con.read_until(b'\x00')[:-1] 
        return con
    con = init_connection()

    data = con.read_until(b'\x00')[:-1]
    data = json.loads(data.decode('utf-8'))
    ID = data['ID']
    print(f"Connected to Spark: {ID} ({dev})")
    # Get the location of this python filem
    path = os.path.dirname(os.path.abspath(__file__))
    pickle_path = os.path.join(path, "offsets_"+ID+".pickle")
    offsets, inverted = pickle.load(open(pickle_path, "rb"))
    num_angles = 7
    max_raw_angle = 2**14
    previous_raw_angles = offsets
    calculated_angles = [0.0]*num_angles

    rospy.init_node(f"Spark_{ID}", anonymous=True)
    pose_publisher = rospy.Publisher(f"Spark_angle/{ID}", Float32MultiArray, queue_size=1)
    enable_publisher = rospy.Publisher(f"Spark_enable/{ID}", Bool, queue_size=1)

    try:
        exit_flag = False
        while not exit_flag:
            try:
                data = con.read_until(b'\x00')[:-1]
            except serial.SerialException:
                print(f"Spark {ID} disconnected")
                rospy.sleep(1)
                con = init_connection()
                continue
            data = json.loads(data.decode('utf-8'))
            raw_angles = data['values']

            if False in data['status']:
                print(f"Spark {ID} has an error in the status: {data['status']}")
            if(previous_raw_angles is not None):
                for i in range(num_angles):
                    # print(get_distance(raw_angles[i], previous_raw_angles[i], max_raw_angle))
                    calculated_angles[i] += inverted[i] * get_distance(raw_angles[i], previous_raw_angles[i], max_raw_angle)
            previous_raw_angles = raw_angles
            pose_publisher.publish(Float32MultiArray(data=calculated_angles))
            enable_publisher.publish(Bool(data['enable_switch']))
    finally:
        con.close()
        print("Connection closed")