import numpy as np
import rospy
from SparkGripper import Gripper
from SparkUR import UR_Direct
from SparkUR_Config import UR_Config
from std_msgs.msg import Float32MultiArray, Bool, Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import signal

# Define a signal handler function
def signal_handler(sig, frame):
    print("Stopped")
    exit()
signal.signal(signal.SIGINT, signal_handler)

# roslaunch perception camera.launch 
# roslaunch dual_arm camera_pose_peract_top.launch 

# roslaunch zeus_moveit_config move_group.launch

# roslaunch dual_arm zeus_bringup.launch
# rosrun dual_arm forward_joint_states.py
# rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNodeLeft.py /tmp/ttyLightning
# rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleControllerLeft.py
# rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNodeRight.py /tmp/ttyThunder
# rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleControllerRight.py 

# Plot -----------------------------------------------------------------
plt.ion()  # Enable interactive mode
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_xlabel('Joints')
ax.set_ylabel('Position Difference')
ax.set_title('Position Difference between UR and Spark Arms')
joint_names = ["Joint {}".format(i) for i in range(1, 7)]
bars_left = ax.bar(joint_names, [0] * 6, label="Left Arm - UR vs. Spark")
bars_right = ax.bar(joint_names, [0] * 6, label="Right Arm - UR vs. Spark", alpha=0.5)
ax.legend()
ax.set_ylim(-np.pi, np.pi)
# ----------------------------------------------------------------------


UR_arms = [None, None]
Grippers = [None, None]
Arm_enable = [False, False]
Gripper_enable = [False, False]


def arm_callback (data):
    # Set the UR arms and grippers to the Spark arms and grippers
    UR_arms[0].set(data.data[0:6])
    UR_arms[1].set(data.data[7:-1])
    # Grippers[0].set(data.data[6]*2)
    # Grippers[1].set(data.data[-1]*2)


    # Calculate the difference between UR and Spark arms
    global position_diff
    ur_positions = [UR_arms[0].get_raw(), UR_arms[1].get_raw()]
    if ur_positions[0] is None or ur_positions[1] is None:
        return
    spark_positions = [data.data[0:6], data.data[7:-1]]

    position_diff = [np.array(ur) - np.array(spark) for ur, spark in zip(ur_positions, spark_positions)]

    for i, bar in enumerate(bars_left):
        bar.set_height(position_diff[0][i])

    for i, bar in enumerate(bars_right):
        bar.set_height(position_diff[1][i])


def enable_callback(data):
    UR_arms[0].set_enable(data.data and Arm_enable[0])
    UR_arms[1].set_enable(data.data and Arm_enable[1])
    # Grippers[0].set_enable(data.data and Gripper_enable[0])
    # Grippers[1].set_enable(data.data and Gripper_enable[1])


def update_plot(frame):
    # Call the arm_callback to update the data
    # arm_callback(data)
    return bars_left + bars_right

def main():
    global UR_controller
    global Griper_controller
    global UR_arms
    global Grippers
    global Arm_enable
    global Gripper_enable
    rospy.init_node('arm_driver')

    # Plot -----------------------------------------------------------------
    ani = FuncAnimation(fig, update_plot, frames=None, blit=True, interval=100)
    plt.show()
    # ----------------------------------------------------------------------

    speed = 0.6
    
    home = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0, 0.0]
    joint_names_left = ["left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_elbow_joint", "left_wrist_1_joint", "left_wrist_2_joint", "left_wrist_3_joint"]
    joint_names_right = ["right_shoulder_pan_joint", "right_shoulder_lift_joint", "right_elbow_joint", "right_wrist_1_joint", "right_wrist_2_joint", "right_wrist_3_joint"]
    UR_arms = [ UR_Direct("/left/", home, speed, joint_names_left),
                UR_Direct("/right/", home, speed, joint_names_right)]
    # Grippers = [Gripper("/left/", binary=False, activate=True),
    #             Gripper("/right/", binary=False, activate=True)]

    # for gripper in Grippers:
    #     gripper.activate()

    rospy.Subscriber('arm_state', Float32MultiArray, arm_callback)
    rospy.Subscriber('enable_state', Bool, enable_callback)
    
    config_r = UR_Config(["/right/"], velocity=True)
    config_l = UR_Config(["/left/"], velocity=True)
    val = True
    while(val!="q"):
        val = input("Press q to quit, ('left, right, both, both_l, both_r, open, close'): ")
        if val == 'q':
            print("Quitting")
        elif val == 'stop':
            config_r.stop()
            config_l.stop()
            print("Stopped")
        elif val == "left":
            Arm_enable = [True, False]
            Gripper_enable = [True, False]
            config_l.unlock_arms()
            print("Left arm enabled")
        elif val == "right":
            Arm_enable = [False, True]
            Gripper_enable = [False, True]
            config_r.unlock_arms()
            print("Right arm enabled")
        elif val == "both":
            Arm_enable = [True, True]
            Gripper_enable = [True, True]
            config_r.unlock_arms()
            config_l.unlock_arms()
            print("Both arms enabled")
        elif val == "both_l":
            Arm_enable = [True, True]
            Gripper_enable = [True, False]
            print("Both arms enabled (Left gripper only)")
        elif val == "both_r":
            Arm_enable = [True, True]
            Gripper_enable = [False, True]
            print("Both arms enabled (Right gripper only)")
        elif val == "open":
            Grippers[0].set_alt(0)
            Grippers[1].set_alt(0)
            print("Both grippers opened")
        elif val == "close":
            Grippers[0].set_alt(1)
            Grippers[1].set_alt(1)
            print("Both grippers closed")
        else:
            if Arm_enable[0]:
                config_l.unlock_arms()
            if Arm_enable[1]:
                config_r.unlock_arms()
            print("Unlocked arms")

if __name__ == "__main__":
    main()
