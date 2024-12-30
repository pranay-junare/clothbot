import rospy
from std_msgs.msg import Float64MultiArray
import controller_manager_msgs.srv

def main():
    rospy.init_node("test_velocity")
    rospy.loginfo("test_velocity node started")
    # Publish to /right/joint_group_vel_controller/command
    pub = rospy.Publisher("/right/joint_group_vel_controller/command", Float64MultiArray, queue_size=1)

    srv_sw_ctrl= rospy.ServiceProxy('/right/controller_manager/switch_controller', controller_manager_msgs.srv.SwitchController)
    sw_ctrl_req= controller_manager_msgs.srv.SwitchControllerRequest()
    sw_ctrl_req.strictness= sw_ctrl_req.STRICT
    sw_ctrl_req.stop_controllers= ['scaled_pos_joint_traj_controller']
    sw_ctrl_req.start_controllers= []
    srv_sw_ctrl(sw_ctrl_req)
    sw_ctrl_req.stop_controllers= []
    sw_ctrl_req.start_controllers= ['joint_group_vel_controller']
    srv_sw_ctrl(sw_ctrl_req)
    
    # UR5e has 6 joints
    msg = Float64MultiArray()
    msg.data = [0.0] * 6
    msg.layout.data_offset = 1

    vel = 0.05
    while not rospy.is_shutdown():
        msg.data = [vel] * 6
        pub.publish(msg)
        rospy.sleep(2.0)
        msg.data = [-vel] * 6
        pub.publish(msg)
        rospy.sleep(2.0)





    rospy.loginfo("test_velocity node ended")

if __name__ == "__main__":
    main()