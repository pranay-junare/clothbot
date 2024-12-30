import numpy as np
import rospy
from SparkGripper import Gripper
from SparkUR_SIM import UR_Effort
from SparkUR_Config import UR_Config
from std_msgs.msg import Float32MultiArray, Bool, Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import signal

UR_arms = [None, None]
Grippers = [None, None]
Arm_enable = [False, False]
Gripper_enable = [False, False]


def arm_callback (data):
    # Set the UR arms and grippers to the Spark arms and grippers
    UR_arms[0].set(data.data[0:6])
    # Grippers[0].set(data.data[6]*2)
    # UR_arms[1].set(data.data[7:-1])
    # Grippers[1].set(data.data[-1]*2)


def enable_callback(data):
    UR_arms[0].set_enable(data.data and Arm_enable[0])
    # Grippers[0].set_enable(data.data and Gripper_enable[0])
    # UR_arms[1].set_enable(data.data and Arm_enable[1])
    # Grippers[1].set_enable(data.data and Gripper_enable[1])

def main():
    global UR_controller
    global Griper_controller
    global UR_arms
    global Grippers
    global Arm_enable
    global Gripper_enable
    rospy.init_node('arm_driver')

    speed = 0.6
    
    home = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0, 0.0]
    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    UR_arms = [ UR_Effort("/", home, speed, joint_names),
                None]
    # Grippers = [Gripper("/left/", binary=False, activate=True),
    #             None]

    # for gripper in Grippers:
    #     gripper.activate()

    rospy.Subscriber('arm_state', Float32MultiArray, arm_callback)
    rospy.Subscriber('enable_state', Bool, enable_callback)

    Arm_enable = [True, False]
    rospy.spin()
    

if __name__ == "__main__":
    main()
