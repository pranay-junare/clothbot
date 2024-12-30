import rospy
import pickle
import moveit_commander
from SparkGripper import Gripper

if __name__ == '__main__':
    rospy.init_node('DemoRecorder')
    rospy.loginfo("DemoRecorder started")

    gripper = Gripper("/right/")
    thunder = moveit_commander.MoveGroupCommander('right_arm')

    with open('thunder_home.pickle', 'rb') as f:
        home = pickle.load(f)

    sequence = []
    sequence_name = input("Enter sequence name: ")
    thunder.set_joint_value_target(home)
    thunder.go()
    gripper.set(255)
    
    command = input("Scene Ready: ")
    pose = thunder.get_current_pose().pose
    gripper_pos = 255
    point_cloud = None
    sequence.append((pose, point_cloud, gripper_pos))

    sub_command = input("Move to next keypoint (Done to exit): ")
    while(sub_command != 'Done'):
        pose = thunder.get_current_pose().pose
        gripper_pos = gripper.get_pos()
        point_cloud = None
        sequence.append((pose, point_cloud, gripper_pos))
        sub_command = input("Move to next keypoint (Done to exit): ")

    
    with open("./recordings/" + sequence_name, 'wb') as f:
        pickle.dump(sequence, f)

    rospy.loginfo("DemoRecorder finished")