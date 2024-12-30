import rospy
import numpy as np
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive


# Go to a joint configuration
class UR_Direct:
    def __init__(self, IP, offset = [0.0]*6, duration=0.8, activate = True, freq = 500,) -> None:
        self.IP = IP
        self.freq = freq
        self.enable = False
        self.offset = offset
        self.duration = duration
        self.force = False
        if(activate):
            self.activate()

    def activate(self):
        self.recieve = RTDEReceive(self.IP)
        self.control = RTDEControl(self.IP, self.freq)
        self.control.zeroFtSensor()
            
    def set(self, angles):
        if(not self.enable):
            return
        position = [angle+offset for angle, offset in zip(angles, self.offset)]
        self.control.servoJ(position, 0.0, 0.0, self.duration, 0.1, 500)
        # return self.recieve.getActualTCPPose()

    def get_raw(self):
        return [angle-offset for angle, offset in zip(self.recieve.getActualQ(), self.offset)]

    def set_enable(self, value):
        self.enable = value

    def get_enable(self):
        return self.enable

    def clear_eStop(self):
        self.control.unlockProtectiveStop()

    def set_force_mode(self, contact):
        if(np.max(contact) > 30 and not self.force):
            print("Force mode")
            task_frame = [0, 0, 0, 0, 0, 0]
            selection_vector = [0,1,0,0,0,0]
            wrench = [0, -0, 0, 0, 0, 0]
            force_type = 1
            limits = [0.1]*6
            limits[1] = 0.5
            self.control.forceMode(task_frame, selection_vector, wrench, force_type, limits)
            # self.control.forceModeSetGainScaling(.5)
            self.control.forceModeSetDamping(.2)
            self.force = True
        elif(np.max(contact) < 10 and self.force):
            print("Stop force mode")
            # self.control.forceModeStop()
            # self.force = False
        else:
            print("No force mode:", np.max(contact), "force:", self.force)

    def get_contact(self):
        # use the tool_contact function to get the contact time:
        return self.recieve.getActualCurrent()
    

class UR_Velocity:
    def __init__(self, IP, offset = [0.0]*6, duration=0.8, freq = 500,) -> None:
        self.recieve = RTDEReceive(IP)
        self.control = RTDEControl(IP, freq)
        self.enable = False
        self.offset = offset[:6]
        self.duration = duration
        self.control.speedStop()
        # getJointTorques
        # getActualQ
        # getActualCurrent
        # getJointControlOutput

            
    def set(self, angles):
        if(not self.enable):
            self.control.speedStop()
            return
        diff = np.array(angles) + np.array(self.offset) - np.array(self.recieve.getActualQ())
        # diff = diff* np.array((1, 0, 0, 0, 0, 0))
        self.control.speedJ(diff *4, 10, 0.05)

    def get_raw(self):
        return [angle-offset for angle, offset in zip(self.recieve.getActualQ(), self.offset)]

    def set_enable(self, value):
        self.enable = value