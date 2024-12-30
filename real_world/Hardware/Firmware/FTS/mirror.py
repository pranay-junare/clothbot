import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray
# cartesian_control_msgs/FollowCartesianTrajectoryActionGoal
from cartesian_control_msgs.msg import FollowCartesianTrajectoryActionGoal
from cartesian_control_msgs.msg import CartesianTrajectoryPoint

from SparkUR_Config import UR_Config

class FT_Arm:
    def __init__ (self, arm_topic):
        self.arm_topic = arm_topic if arm_topic[-1] == '/' else arm_topic + '/'
        rospy.Subscriber(self.arm_topic + "wrench", WrenchStamped, self.FT_callback)
        self.wrench = None

    def FT_callback(self, msg):
        # print(msg)
        self.wrench = msg.wrench

    def get_wrench(self):
        return self.wrench
    
    def is_ready(self):
        return self.wrench is not None


class EEF_Arm:
    def __init__ (self, arm_topic): # TODO: Enable false
        self.arm_topic = arm_topic if arm_topic[-1] == '/' else arm_topic + '/'
        scaled_pos_joint_traj_controller = self.arm_topic + "pose_based_cartesian_traj_controller/follow_cartesian_trajectory/goal"
        self.controller = rospy.Publisher(scaled_pos_joint_traj_controller, FollowCartesianTrajectoryActionGoal, queue_size=1)
        self.msg = FollowCartesianTrajectoryActionGoal()
        self.enable = True
        self.offset = offset=[0]*6
        self.actual = None
        # self.query_state = rospy.ServiceProxy(self.arm_topic+"/scaled_pos_joint_traj_controller/query_state", QueryTrajectoryState)
        
    def set(self, xyzrpyw):
        if self.enable:
            xyzrpyw = [angle+offset for angle, offset in zip(xyzrpyw, self.offset)]
            self.msg.goal.trajectory.points = [CartesianTrajectoryPoint()]
            self.msg.goal.trajectory.points[0].pose.position.x = xyzrpyw[0]
            self.msg.goal.trajectory.points[0].pose.position.y = xyzrpyw[1]
            self.msg.goal.trajectory.points[0].pose.position.z = xyzrpyw[2]
            self.msg.goal.trajectory.points[0].pose.orientation.x = xyzrpyw[3]
            self.msg.goal.trajectory.points[0].pose.orientation.y = xyzrpyw[4]
            self.msg.goal.trajectory.points[0].pose.orientation.z = xyzrpyw[5]
            # self.msg.goal.trajectory.points[0].pose.orientation.w = xyzrpyw[6]
            self.msg.goal.trajectory.points[0].time_from_start = rospy.Duration(10)
            self.msg.goal.trajectory.header.stamp = rospy.Time.now()
            self.controller.publish(self.msg)
        else:
            print("Arm is not enabled")


    def set_enable(self, value):
        self.enable = value

    # def get_state(self):
    #     return self.query_state(rospy.Time.now())




def main():
    rospy.init_node('FT_Arm', anonymous=True)
    conf = UR_Config(["/left/", "/right/"])
    conf.unlock_arms()
    conf.cart_mode()

    left_arm = EEF_Arm("/left/")
    left_ft = FT_Arm("/left/")
    right_arm = EEF_Arm("/right/")
    right_ft = FT_Arm("/right/")
    while not left_ft.is_ready() or not right_ft.is_ready():
        print("Waiting for: %s, %s" % (left_arm.arm_topic, right_arm.arm_topic), end='\r')
        rospy.sleep(.5)
    # print(left_ft.get_wrench())
    # print(right_ft.get_wrench())

    while not rospy.is_shutdown():
        # pos = list(left_arm.get_state().position)
        # X, Y, Z, RX, RY, RZ, W = pos
        pos = [100, 100, 100, 3.14, 0, 0, 0 , 0]
        # print(pos, end='\r')
        pos[0] += 0.1
        left_arm.set(pos)
        print("Running")
        rospy.sleep(4)
        exit()
    
if __name__ == "__main__":
    main()
