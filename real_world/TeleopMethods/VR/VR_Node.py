import triad_openvr
import os
import sys
import rospy
from std_msgs.msg import Float32MultiArray


def main():
    sys.stdout = open(os.devnull, "w")
    vr = triad_openvr.triad_openvr()
    sys.stdout = sys.__stdout__

    print("Connected to VR")
    rospy.init_node('VR_Node', anonymous=True)
    thunder_pub = rospy.Publisher('VR/thunder', Float32MultiArray, queue_size=1)
    lightning_pub = rospy.Publisher('VR/lightning', Float32MultiArray, queue_size=1)
    pubs = {}
    rate = rospy.Rate(30)

    while(True):
        for controller in ("controller_1", "controller_2"):
            if controller in vr.devices:
                if controller not in pubs:
                    devices = vr.devices[controller]
                    buttons = devices.get_controller_inputs()
                    if (buttons['trackpad_x'] < -0.5):
                        pubs[controller] = thunder_pub
                        print(f"Thunder connected to VR: {controller}")
                    elif (buttons['trackpad_x'] > 0.5):
                        pubs[controller] = lightning_pub
                        print(f"Lightning connected to VR: {controller}")
                
                if controller in pubs:
                    devices = vr.devices[controller]
                    buttons = devices.get_controller_inputs()
                    pos = devices.get_pose_euler()
                    if buttons is not None and pos is not None:
                        pos_xyz = pos[:3]
                        pos_rpy = [angle*3.14159/180 for angle in pos[3:]]
                        data = pos_xyz + pos_rpy + [buttons['trigger'], buttons['menu_button'], buttons['grip_button'], 
                                                    buttons['trackpad_x'], buttons['trackpad_y']]
                        pubs[controller].publish(Float32MultiArray(data=data))
        rate.sleep()

if __name__ == '__main__':
    main()