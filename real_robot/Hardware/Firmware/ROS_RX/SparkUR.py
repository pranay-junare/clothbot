import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.srv import QueryTrajectoryState


# Go to a joint configuration
class UR_Direct:
    def __init__(self, topic, offset = [0.0]*6, duration=0.8, joint_names=None) -> None:
        self.topic = topic if topic[-1] == '/' else topic + '/'
        self.pub = rospy.Publisher(self.topic+"scaled_pos_joint_traj_controller/command", JointTrajectory, queue_size=1)
        rospy.Subscriber(self.topic+"joint_states", JointState, self.callback)
        self.msg = JointTrajectory()
        self.enable = False
        self.offset = offset
        self.duration = duration
        self.msg.joint_names = joint_names
        self.URpos = None 
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

    def get_raw(self):
        return [angle-offset for angle, offset in zip(self.URpos, self.offset)]
    
    def callback(self, msg):
        self.URpos = msg.position
        

    def set_enable(self, value):
        self.enable = value

# ---------------------------------------------

class UR_Velocity:
    def __init__(self, topic, offset = [0.0]*6) -> None:
        self.topic = topic if topic[-1] == '/' else topic + '/'
        self.URpos = None
        self.callback = lambda msg: setattr(self, 'URpos', msg.position) 
        self.pub = rospy.Publisher(self.topic+"joint_group_vel_controller/command", Float64MultiArray, queue_size=1)
        self.sub = rospy.Subscriber(self.topic+"joint_states", JointState, self.callback)
        self.enable = False
        self.offset = offset
        self.msg = Float64MultiArray()
        self.last_time = rospy.Time.now()
        self.last_diff = [0.0]*6
 
    def set(self, angles):
        if self.URpos is None: return

        position = [angle+offset for angle, offset in zip(angles, self.offset)]

        last_time = self.last_time
        last_diff = self.last_diff
        time = rospy.Time.now()
        diff = [angle - pos for angle, pos in zip(position, self.URpos)]

        self.last_time = time
        self.last_diff = diff

        accels = [dif - last_dif for dif, last_dif in zip(diff, last_diff)]
        # time_delta  = (time - last_time).to_sec()
        # max_accel = 0.1
        # for i in range(3):
        #     if accels[i] > max_accel*time_delta:
        #         diff[i] = last_diff[i] + max_accel * time_delta
        

        # Scaling
        self.msg.data = [4* dif * np.abs(dif) for dif in diff]
        self.msg.data = [max(min(dif, 10), -10) for dif in self.msg.data]
        # self.msg.data = [angle if abs(angle) > 0.01 else 0.0 for angle in self.msg.data]
        
        
        # Enable / Disable
        if(not self.enable): self.msg.data = [0.0]*6
        for i in range(3):
            self.msg.data[i] *= 0.0
        if self.enable:
            # print(self.msg.data)
            print(accels)
        self.pub.publish(self.msg)
        self.time = time

    def set_enable(self, value):
        self.enable = value
