import rtde_control
import rtde_receive
from gripper import RobotiqGripper

# https://sdurobotics.gitlab.io/ur_rtde/api/api.html
ip = "192.168.0.102"
r = rtde_receive.RTDEReceiveInterface(ip)
c = rtde_control.RTDEControlInterface(ip, 500)

pose = r.getActualTCPPose()

c.moveL(pose, 0.1, 0.1)

grip = RobotiqGripper()
grip.connect(ip, 63352)
grip.activate()
grip.set_enable(True)

grip.set(255)
pose[0] += 0.1
c.moveL(pose, 0.1, 0.1)

grip.set(0)
pose[0] -= 0.1
c.moveL(pose, 0.1, 0.1)
