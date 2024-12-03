import rospy
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input as inputMsg

# rACT: Means the activation of the gripper.
# rGTO: Means the goal position request.
# rPR: Means the requested position.
# rSP: Means the speed at which the gripper will try to achieve the requested position.
# rFR: Means the force applied on the object by the gripper.

# gACT: Means the activation status of the gripper.
# gPO: Means the actual position of the gripper.
# gCU: Means the current drawn by the gripper.
# gPr: Means the current position of the fingers.
# gFLT: Means the fault status of the gripper.
# gGTO: Means the status of the goal position request.
# gSTA: Means the status of the gripper.
# gOBJ: Means the status of the object detection.



class Gripper:
    def __init__(self, topic, force = 150, binary=False, activate=True) -> None:
        self.topic = topic + "/" if topic[-1] != "/" else topic
        self.pub = rospy.Publisher(self.topic + "Robotiq2FGripperRobotOutput", outputMsg, queue_size=1)
        self.msg = outputMsg()
        self.force = force
        self.enable = True
        self.binary = binary
        
        

    def activate(self):
        self.msg.rACT = 1
        self.msg.rGTO = 1
        self.msg.rSP = 255
        self.msg.rFR = self.force
        self.pub.publish(self.msg)


    
    def open(self):
        if(not self.enable):
            return
        self.msg.rPR = 0
        self.pub.publish(self.msg)

    def close(self):
        if(not self.enable):
            return
        self.msg.rPR = 255
        self.pub.publish(self.msg)

    def set(self, pos):
        if(not self.enable):
            return
        self.set_alt(pos)
        
    def set_alt(self, pos):
        if(self.binary):
            pos = 0 if pos < 0.1 else 255
        self.msg.rPR = max(0, min(255, int(pos*255)))
        self.pub.publish(self.msg)

    def set_enable(self, value):
        self.enable = value

    # def get_pos(self):
    #     return rospy.wait_for_message(self.topic + "Robotiq2FGripperRobotOutput", outputMsg).rPR
