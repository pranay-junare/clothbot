import rospy
import std_srvs.srv
import ur_dashboard_msgs.msg
import ur_dashboard_msgs.srv
import controller_manager_msgs.srv

class UR_Config:
    def __init__(self, arms, velocity=False) -> None:
        append_slash = lambda x: x if x[-1] == '/' else x + '/'
        self.arms = [append_slash(arm) for arm in arms]
        # https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/395c0541b20d0da2cd480e2ad85b2100410fb043/ur_robot_driver/doc/ROS_INTERFACE.md
        self.unlock = []
        self.play_serv = []
        self.pause_serv = []
        self.stop_serv = []
        self.mode_serv = []
        self.safty_close = []
        for arm in arms:
            rospy.ServiceProxy(arm+'ur_hardware_interface/dashboard/load_program', ur_dashboard_msgs.srv.Load)('external.urp')
            self.unlock.append(rospy.ServiceProxy(arm+'ur_hardware_interface/dashboard/unlock_protective_stop', std_srvs.srv.Trigger))
            self.play_serv.append(rospy.ServiceProxy(arm+'ur_hardware_interface/dashboard/play', std_srvs.srv.Trigger))
            self.pause_serv.append(rospy.ServiceProxy(arm+'ur_hardware_interface/dashboard/pause', std_srvs.srv.Trigger))
            self.stop_serv.append(rospy.ServiceProxy(arm+'ur_hardware_interface/dashboard/stop', std_srvs.srv.Trigger))
            self.mode_serv.append(rospy.ServiceProxy(arm+'controller_manager/switch_controller', controller_manager_msgs.srv.SwitchController))
            self.safty_close.append(rospy.ServiceProxy(arm+'ur_hardware_interface/dashboard/close_popup', std_srvs.srv.Trigger))
            rospy.Subscriber(arm+'ur_hardware_interface/safety_mode', ur_dashboard_msgs.msg.SafetyMode, self.safety_callback)
        if velocity:
            self.vel_mode()

    def safety_callback(self, msg):
        # NORMAL=1 REDUCED=2 PROTECTIVE_STOP=3 RECOVERY=4 SAFEGUARD_STOP=5 SYSTEM_EMERGENCY_STOP=6
        # ROBOT_EMERGENCY_STOP=7 VIOLATION=8 FAULT=9 VALIDATE_JOINT_ID=10 UNDEFINED_SAFETY_MODE=11
        # AUTOMATIC_MODE_SAFEGUARD_STOP=12 SYSTEM_THREE_POSITION_ENABLING_STOP=13
        # rospy.loginfo(f'Safety mode: {msg.mode}')
        pass
            
    def unlock_arms(self):
        for close in self.safty_close:
            close()
        rospy.sleep(1)
        for unlock in self.unlock:
            unlock()
        rospy.sleep(1)
        for play in self.play_serv:
            play()
        rospy.sleep(1)

    def stop(self):
        for stop in self.stop_serv:
            stop()

    def vel_mode(self):
        for mode in self.mode_serv:
            sw_ctrl_req= controller_manager_msgs.srv.SwitchControllerRequest()
            sw_ctrl_req.strictness= sw_ctrl_req.STRICT
            sw_ctrl_req.stop_controllers= ['scaled_pos_joint_traj_controller']
            sw_ctrl_req.start_controllers= []
            mode(sw_ctrl_req)
            sw_ctrl_req.stop_controllers= []
            sw_ctrl_req.start_controllers= ['joint_group_vel_controller']
            mode(sw_ctrl_req)

    def pos_mode(self): 
        for mode in self.mode_serv:
            sw_ctrl_req= controller_manager_msgs.srv.SwitchControllerRequest()
            sw_ctrl_req.strictness= sw_ctrl_req.STRICT
            sw_ctrl_req.stop_controllers= ['joint_group_vel_controller']
            sw_ctrl_req.start_controllers= []
            mode(sw_ctrl_req)
            sw_ctrl_req.stop_controllers= []
            sw_ctrl_req.start_controllers= ['scaled_pos_joint_traj_controller']
            mode(sw_ctrl_req)

    def cart_mode(self):
        for mode in self.mode_serv:
            sw_ctrl_req= controller_manager_msgs.srv.SwitchControllerRequest()
            sw_ctrl_req.strictness= sw_ctrl_req.STRICT
            sw_ctrl_req.stop_controllers= ['joint_group_vel_controller']
            sw_ctrl_req.start_controllers= []
            mode(sw_ctrl_req)
            sw_ctrl_req.stop_controllers= []
            sw_ctrl_req.start_controllers= ['pose_based_cartesian_traj_controller']
            mode(sw_ctrl_req)

if __name__ == '__main__':
    config = UR_Config(["/left/", "/right/"])
    while val := input("Press u, p, vel, or pos: "):
        print(val)
        if val == 'u':
            config.unlock_arms()
        elif val == 'p':
            config.pause()
        elif val == 'pos':
            config.pos_mode()
        elif val == 'vel':
            config.vel_mode()
        elif val == 'cart':
            config.cart_mode()

        else:
            print("Invalid input")