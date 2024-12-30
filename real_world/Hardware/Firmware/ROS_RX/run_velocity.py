import numpy as np
import rospy
from SparkGripper import Gripper
from SparkUR import UR_Direct, UR_Velocity
from SparkUR_Config import UR_Config
from std_msgs.msg import Float32MultiArray, Bool, Float32

# nc 192.168.0.102 29999 ; load external.urp ; play 

# roslaunch perception camera.launch 
# roslaunch dual_arm camera_pose_peract_top.launch

# rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /tmp/ttyUR
# rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py

# roslaunch dual_arm zeus_bringup.launch
# roslaunch zeus_moveit_config move_group.launch
# rosrun dual_arm forward_joint_states.py
# rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNodeLeft.py /tmp/ttyLightning
# rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNodeRight.py /tmp/ttyThunder
# rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleControllerRight.py 



UR_controller = None
UR_arms = [None, None]
Griper_controller = None
Grippers = [None, None]

def arm_callback (data):
    UR_controller.set(data.data[0:6])
    # print(data.data, end='\r')
   
def enable_callback(data):
    UR_controller.set_enable(data.data)
    Griper_controller.set_enable(data.data)

def gripper_callback(data):
    Griper_controller.set(int((1-data.data)*255))
    
def swap_callback(data):
    global UR_controller
    global Griper_controller
    if data.data:
        UR_controller = UR_arms[1]
        Griper_controller = Grippers[1]
    else:
        UR_controller = UR_arms[0]
        Griper_controller = Grippers[0]



def main():
    global UR_controller
    global Griper_controller
    global UR_arms
    global Grippers
    rospy.init_node('arm_driver')

    UR_arms = [ UR_Velocity("/left/",
                          [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0, 0.0]),
                UR_Velocity("/right/",
                          [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0, 0.0])]
    UR_controller = UR_arms[1]
    Grippers = [Gripper("/left/", binary=False),
                Gripper("/right/", binary=False)]
    Griper_controller = Grippers[1]

    for gripper in Grippers:
        gripper.activate()

    rospy.Subscriber('arm_state', Float32MultiArray, arm_callback)
    rospy.Subscriber('enable_state', Bool, enable_callback)
    rospy.Subscriber('gripper_state', Float32, gripper_callback)
    # rospy.Subscriber('swap_arms', Bool, swap_callback)
    
    config = UR_Config(["/right/"])
    val = True
    while(val!="q"):
        val = input("Press q to quit: ")
        config.unlock_arms()

if __name__ == "__main__":
    main()
