import rospy
from std_msgs.msg import Bool

def callback(data):
    global arm_pos
    arm_pos = data.data

if __name__ == '__main__':
    global arm_pos
    rospy.init_node('arm_state_subscriber')
    pub = rospy.Publisher('swap_arms', Bool, queue_size=10)
    arm = False

    user_data = input("Enter 'Done' to exit: ")
    while user_data != "Done":
        pub.publish(Bool(data=arm))
        arm = not arm
        user_data = input("Enter 'Done' to exit: ")
 
        