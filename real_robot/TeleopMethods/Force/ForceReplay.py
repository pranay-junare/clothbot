import rospy
from std_msgs.msg import Float32MultiArray
import time
import pickle
import sys
import os

def play_pickle(input, pub):
    message = Float32MultiArray()
    with open(input, 'rb') as f:
        trajectory = pickle.load(f)
    time_start = rospy.get_time()
    for t, data in trajectory:
        while (rospy.get_time() - time_start) < t:
            time.sleep(0.001)
        message.data = data
        pub.publish(message)


if __name__ == '__main__':
    rospy.init_node('ForceReplay', anonymous=True)
    pub = rospy.Publisher('/thunder_replay_eef', Float32MultiArray, queue_size=10)
    input_dir = sys.argv[1]

    for file in os.listdir(input_dir):
        if file.endswith(".pickle"):
            print(f"Running: {input_dir}/{file}")
            play_pickle(os.path.join(input_dir,file), pub)
            exit()