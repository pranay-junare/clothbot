import pyspacemouse
import time
import os
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

def main():
    dev = os.sys.argv[1]
    print("Connecting to SpaceMouse: "+dev)
    sys.stdout = open(os.devnull, "w")
    rospy.init_node('SpaceMouse'+dev[-1], anonymous=True)
    thunder = rospy.Publisher('SpaceMouseThunder', Float32MultiArray, queue_size=1)
    thunder_log = rospy.Publisher('SpaceMouseThunderLog', String, queue_size=10)
    lightning = rospy.Publisher('SpaceMouseLightning', Float32MultiArray, queue_size=1)
    lightning_log = rospy.Publisher('SpaceMouseLightningLog', String, queue_size=10)
    success = pyspacemouse.open(path=dev)
    sys.stdout = sys.__stdout__
    mod = 50
    count = 0
    if success:
        pub = None
        log = None
        while 1:
            time.sleep(0.0001)
            state = pyspacemouse.read()
            count += 1
            if pub is None:
                if state.buttons[0] == 1:
                    pub = thunder 
                    log = thunder_log
                    log.publish("Thunder connected to SpaceMouse: "+dev)
                    print("Thunder connected to SpaceMouse: "+dev)
                elif state.buttons[1] == 1:
                    pub = lightning 
                    log = lightning_log
                    log.publish("Lightning connected to SpaceMouse: "+dev)
                    print("Lightning connected to SpaceMouse: "+dev)
                count = 0
            else:
                if count == mod:
                    count = 0
                    data =[state.x, state.y, state.z, state.roll, state.pitch, state.yaw] + state.buttons
                    pub.publish(Float32MultiArray(data=data))

if __name__ == '__main__':
    main()