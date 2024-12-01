import rospy
from std_msgs.msg import Float32MultiArray

def force_torque_callback(data):
    global force_torque
    force_torque = data.data
    print("UPDATE")

sub = rospy.Subscriber("/lightning_ft", Float32MultiArray, force_torque_callback)

rospy.init_node('test_ros')
rospy.spin()