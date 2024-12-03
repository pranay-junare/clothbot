import pybullet as pb
import time

# launch pybullet enviroment with nothing in it
client = pb.connect(pb.GUI, options="--width=1920 --height=1080")
pb.setGravity(0, 0, -10)
pb.setRealTimeSimulation(True)
# load urdf
pb.loadURDF("./dual_arm.urdf", useFixedBase = True)
# wait
for _ in range(2400):
    pb.stepSimulation()
    time.sleep(1.0 / 240.0)



