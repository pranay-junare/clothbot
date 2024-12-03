import robotiq_gripper
import time

ip = "192.168.0.102"

def log_info(gripper):
    print(f"Pos: {str(gripper.get_current_position()): >3}  "
          f"Open: {gripper.is_open(): <2}  "
          f"Closed: {gripper.is_closed(): <2}  ")

print("Creating gripper...")
gripper = robotiq_gripper.RobotiqGripper()
print("Connecting to gripper...")
gripper.connect(ip, 63352)
print("Activating gripper...")
gripper.activate()

print("Testing gripper...")
print(gripper.move_and_wait_for_pos(255, 255, 255))
time.sleep(1)
