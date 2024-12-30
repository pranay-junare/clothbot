import numpy as np
import rospy
from SparkUR_RTDE import UR_Direct, UR_Velocity
from std_msgs.msg import Float32MultiArray, Bool
import robotiq_gripper
import datetime
import signal

def signal_handler(sig, frame):
    print("Stopped")
    exit()

signal.signal(signal.SIGINT, signal_handler)

def arm_callback (data):
    enable_threshold = 2.0
    # Set the UR arms and grippers to the Spark arms and grippers
    # lighting_gripper_state = np.clip((1-data.data[6]*2), 0, 1)
    thunder_gripper_state = np.clip((data.data[6]*2+0.8), 0, 1)
    
    binary = True
    if(binary):
        # lighting_gripper_state = float(round(lighting_gripper_state))
        thunder_gripper_state = float(round(thunder_gripper_state))


    # if Lightning_Run:
    #     lighting_real = Lightning_UR.get_raw()
    #     diff = np.array(data.data[0:6]) - np.array(lighting_real)
    #     Lightning_DiffPub.publish(Float32MultiArray(data=list(diff) + [lighting_gripper_state]))
    #     if(np.max(np.abs(diff)) > enable_threshold and Lightning_UR.get_enable()):
    #         print(datetime.datetime.now(), "Lightning diff:", np.max(np.abs(diff)))
    #     else:
    #         lightning_eef = Lightning_UR.set(data.data[0:6])
    #         lightning_grp = Lightning_Gripper.set(int(255*lighting_gripper_state))

    if Thunder_Run:
        thunder_real = Thunder_UR.get_raw()
        diff = np.array(data.data[0:6]) - np.array(thunder_real)
        Thunder_DiffPub.publish(Float32MultiArray(data=list(diff) + [thunder_gripper_state]))
        if (np.max(np.abs(diff)) > enable_threshold and Thunder_UR.get_enable()):
            print(datetime.datetime.now(), "Thunder diff:", diff)
        else: 
            thunder_eef = Thunder_UR.set(data.data[0:6])
            thunder_grp = Thunder_Gripper.set(int(255*thunder_gripper_state))

def enable_callback(data):
    # Lightning_UR.set_enable(data.data and Lightning_Run)
    # Lightning_Gripper.set_enable(data.data and Lightning_Run)

    Thunder_UR.set_enable(data.data and Thunder_Run)
    Thunder_Gripper.set_enable(data.data and Thunder_Run)

def main():
    # global Lightning_Run
    # global Lightning_UR
    # global Lightning_Gripper
    # global Lightning_DiffPub
    
    global Thunder_Run
    global Thunder_UR
    global Thunder_Gripper
    global Thunder_DiffPub

    rospy.init_node('arm_driver')

    speed = .2
    # Lightning_Run = True
    Thunder_Run = True

    home = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0, 0.0]

    # lightning_ip = "192.168.0.102" # Lightning
    # Lightning_UR = UR_Direct(lightning_ip, offset=home, duration=speed, activate=Lightning_Run)
    # Lightning_Gripper = robotiq_gripper.RobotiqGripper()
    # if(Lightning_Run):
    #     Lightning_Gripper.connect(lightning_ip, 63352)
    #     Lightning_Gripper.activate()

    thunder_ip = "192.168.0.101" # Thunder
    Thunder_UR = UR_Direct(thunder_ip, offset=home, duration=speed, activate=Thunder_Run)
    Thunder_Gripper = robotiq_gripper.RobotiqGripper()
    if(Thunder_Run):
        Thunder_Gripper.connect(thunder_ip, 63352)
        Thunder_Gripper.activate()

    
    # Lightning_DiffPub = rospy.Publisher('lightning_diff', Float32MultiArray, queue_size=10)
    Lightning_ENPub = rospy.Publisher('lightning_enable', Bool, queue_size=10)
    Thunder_DiffPub = rospy.Publisher('thunder_diff', Float32MultiArray, queue_size=10)
    Thunder_ENPub = rospy.Publisher('thunder_enable', Bool, queue_size=10)
    rospy.Subscriber('arm_state', Float32MultiArray, arm_callback)
    rospy.Subscriber('enable_state', Bool, enable_callback)
    rospy.sleep(0.5)

    val = "stop"
    while(val!="q"):
        if val == 'q':
            print("Quitting")
        elif val == "stop":
            # Lightning_Run = False
            Thunder_Run = False
            print("Stopped")
        elif val == "lightning":
            # Lightning_Run = True
            Thunder_Run = False
            print("Lightning enabled")
        elif val == "thunder":
            # Lightning_Run = False
            Thunder_Run = True
            print("Thunder enabled")
        elif val == "both":
            # Lightning_Run = True
            Thunder_Run = True
            print("Both enabled")
        elif val == "open":
            # if Lightning_Run:
            #     Lightning_Gripper.set_now(0)
            if Thunder_Run:
                Thunder_Gripper.set_now(0)
            print("Opened grippers")
        elif val == "close":
            # if Lightning_Run:
            #     Lightning_Gripper.set_now(255)
            if Thunder_Run:
                Thunder_Gripper.set_now(255)
            print("Closed grippers")
        elif val == "":
            # if Lightning_Run:
            #     Lightning_UR.clear_eStop()
            if Thunder_Run:
                Thunder_UR.clear_eStop()
            print("Cleared eStop")
        else:
            print("Invalid input")
        # Lightning_ENPub.publish(Lightning_Run)
        Thunder_ENPub.publish(Thunder_Run)
        rospy.sleep(.1)
        val = input("Press q to quit, ('Stop', 'Lightning', 'Thunder', 'Both', 'Open', 'Close')\n").lower()
        print()

     
    
if __name__ == "__main__":
    main()
