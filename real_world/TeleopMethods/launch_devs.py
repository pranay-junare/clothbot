import os
import subprocess
import time
import rospy
import atexit

def get_devs():
    cmd = "udevadm info --name="
    devs = []
    spark_devs = []
    SM_devs = []
    VR_devs = []
    haptic_devs = []
    for dev in os.listdir('/dev'):
        if 'ttyUSB' in dev:
            devs.append(dev)
        if 'hidraw' in dev:
            devs.append(dev)
        if 'ttyACM' in dev:
            devs.append(dev)
    for dev in devs:
        cmd = "udevadm info --name=/dev/" + dev + " --attribute-walk"
        output = os.popen(cmd).read()
        if "cp210x" in output:
            print("Spark Device found: " + dev)
            spark_devs.append(os.path.join("/dev/", dev))
        if "3Dconnexion" in output:
            print("SpaceMouse Device found: " + dev)
            SM_devs.append(os.path.join("/dev/", dev))
        if "STMicroelectronics" in output:
            print("Haptic Device found: " + dev)
            haptic_devs.append(os.path.join("/dev/", dev))
    if os.path.exists('/dev/serial/by-id/usb-HTC_Hub_Controller-if00'):
        print("VR Device found")
        VR_devs.append('/dev/serial/by-id/usb-HTC_Hub_Controller-if00')

    return spark_devs, SM_devs, VR_devs, haptic_devs

def cleanup(modules, arms):
    for module in modules:
        module.kill()
    print("Exiting")

def StartModules(Spark_devs, SM_devs, VR_devs, haptic_devs):
    print("Starting modules---------------------")
    modules = [subprocess.Popen(['roscore'], start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)]
    # time.sleep(1)
    rospy.init_node('Main', anonymous=True)

    modules.append(subprocess.Popen(['python3', 'Force/ForceNode.py'], start_new_session=True))
    modules.append(subprocess.Popen(['python3', 'realsense/realsense.py', '/both/front/'], start_new_session=True))
    modules.append(subprocess.Popen(['python3', 'realsense/realsense.py', '/both/top/'], start_new_session=True))
    # modules.append(subprocess.Popen(['python3', 'realsense/realsense.py', '/lightning/wrist/'], start_new_session=True))
    # modules.append(subprocess.Popen(['python3', 'realsense/realsense.py', '/thunder/wrist/'], start_new_session=True))
    
    path = os.path.dirname(os.path.abspath(__file__))
    for vr in VR_devs:
        modules.append(subprocess.Popen(['python3', 'VR/VR_Node.py'], start_new_session=True))
    for dev in Spark_devs:
        modules.append(subprocess.Popen(['python3', os.path.join(path, 'Spark/SparkNode.py'), dev], start_new_session=True))
    for dev in SM_devs:
        modules.append(subprocess.Popen(['python3', os.path.join(path, 'SM/SpaceMouseROS.py'), dev], start_new_session=True))
    for dev in haptic_devs:
        modules.append(subprocess.Popen(['python3', os.path.join(path, 'Haptic/HapticNode.py'), dev], start_new_session=True))
    time.sleep(8)
    print("Modules started----------------------")
    return modules


def main():
    Spark_devs, SM_devs, VR_devs, haptic_devs = get_devs()
    modules = StartModules(Spark_devs, SM_devs, VR_devs, haptic_devs)
    atexit.register(cleanup, modules, None)
    rospy.spin()

if __name__ == '__main__':
    main()
