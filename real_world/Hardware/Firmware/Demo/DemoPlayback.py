import rospy
import pickle
import moveit_commander
from SparkGripper import Gripper


if __name__ == '__main__':
    rospy.init_node('DemoPlayback')
    rospy.loginfo("Playback started")

    gripper = Gripper("/right/")
    gripper.activate()

    
    thunder = moveit_commander.MoveGroupCommander('right_arm')
    sequence_name = input("Enter sequence name: ")
    with open("./recordings/" + sequence_name, 'rb') as f:
        sequence = pickle.load(f)
    
    for pose, point_cloud, gripper_pos in sequence:
        print(gripper_pos)
        input("Press enter to continue")
        thunder.set_pose_target(pose)
        thunder.go()
        # plan, fraction = thunder.compute_cartesian_path([pose], 0.01, 0.0)
        # thunder.execute(plan)
        gripper.set(gripper_pos)
        