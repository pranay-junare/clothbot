import rospy
from std_msgs.msg import Float32MultiArray, Bool
import os
import subprocess
import sys

policy_mode = False
policy_data = [0, 0, 0, 0]
def callback(data):
    global policy_data
    policy_data = data.data

if __name__ == '__main__':
    rospy.init_node('ForceNode', anonymous=True)
    pub_axis = rospy.Publisher('/Force/force_ctl', Float32MultiArray, queue_size=10)
    pub_start = rospy.Publisher('/Force/start', Bool, queue_size=10)
    pub_stop = rospy.Publisher('/Force/stop', Bool, queue_size=10)
    rospy.Subscriber('/Force/policy', Float32MultiArray, callback)

    # Check for joystick
    sys.stdout = open(os.devnull, "w")
    sys.stderr = open(os.devnull, "w")
    import pygame
    pygame.init()
    sys.stdout = sys.__stdout__
    sys.stderr = sys.__stderr__
    if not pygame.joystick.get_init():
        pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No Joystick found")
        exit()
    print(f"Joystick found: {pygame.joystick.get_count()}")
    joystick = pygame.joystick.Joystick(pygame.joystick.get_count()-1)
    joystick.init()

    started = False
    name = None
    axis_msg = Float32MultiArray()
    while not rospy.is_shutdown():
        pygame.event.get()

        time = rospy.get_time()
        if joystick.get_button(1) and not started:
            pub_start.publish(Bool(True))
            # os.system(f"rosbag record --output-name=/tmp/demo_{time} -a __name:=force_bag")
            # Using subprocess.Popen
            print("\tStarting recording")
            rospy.sleep(4)
            name = f"/tmp/demo_{time}"
            subprocess.Popen(['rosbag', 'record', f"--output-name={name}", '-a', '__name:=force_bag'])
            rospy.sleep(2)
            print(f"\tRecording started at {time}")
            started = True
        if joystick.get_button(0) and started: # Stop recording
            os.system("rosnode kill force_bag")
            print(f"Recording stopped at {time}")
            rospy.sleep(2)
            pub_stop.publish(Bool(True))
            started = False
        if joystick.get_button(7): # Copy bag file
            if name:
                print(f"Copying {name}.bag to /home/rpmdt05/Aurora/Force")
                os.system(f"cp {name}.bag /home/rpmdt05/Aurora/Force")
                # Print MD5 sum for both files
                file_name = name.split('/')[-1]
                os.system(f"md5sum {name}.bag")
                os.system(f"md5sum /home/rpmdt05/Aurora/Force/{file_name}.bag")
                name = None
            
        if joystick.get_button(2) and policy_mode:
            policy_mode = False
            print("Joystick mode")
        if joystick.get_button(3) and not policy_mode:
            policy_mode = True
            print("Policy mode")
            
        joystick_data = [joystick.get_axis(0), joystick.get_axis(1), joystick.get_axis(2), joystick.get_axis(3)]
        if policy_mode:
            axis_msg.data = policy_data
        else:
            axis_msg.data = joystick_data
        pub_axis.publish(axis_msg)


        rospy.sleep(0.1)