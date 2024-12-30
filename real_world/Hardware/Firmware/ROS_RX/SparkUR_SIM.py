import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Go to a joint configuration
class UR_Effort:
    def __init__(self, topic, offset = [0.0]*6, duration=0.8, joint_names=None) -> None:
        self.topic = topic if topic[-1] == '/' else topic + '/'
        self.pub = rospy.Publisher(self.topic+"eff_joint_traj_controller/command", JointTrajectory, queue_size=1)
        print(self.topic+"eff_joint_traj_controller/command")
        self.msg = JointTrajectory()
        self.enable = False
        self.offset = offset
        self.duration = duration
        self.msg.joint_names = joint_names
        if joint_names is None:
            self.msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            
    def set(self, angles):
        self.msg.header.stamp = rospy.Time.now()
        self.frame_id = "base_link"
        position = [angle+offset for angle, offset in zip(angles, self.offset)]
        # self.msg.points = [JointTrajectoryPoint(positions=position, velocities=[0.0]*6, accelerations=[0.0]*6, time_from_start=rospy.Duration(0.5))]
        self.msg.points = [JointTrajectoryPoint(positions=position, velocities=[0.0]*6, accelerations=[0.0]*6, time_from_start=rospy.Duration(self.duration))]
        if(not self.enable):
            return
        self.pub.publish(self.msg)

    def set_enable(self, value):
        self.enable = value
