import rospy
from std_msgs.msg import Bool, Float32MultiArray
import argparse
import pickle

def eval():
    with open("./offsets/offsets.pkl", "rb") as f:
        offsets = pickle.load(f)
        # for i, offset in enumerate(offsets):
        #     print(f"{i}: {offset}")
    # offsets = [(0,0)]*5
    

    try:
        print("Initializing Node...")
        rospy.init_node("EvalOffsets", anonymous=True)
        print("Node Initialized")
    except rospy.ROSException as e:
        print(f"Failed to initialize node: {e}")
        return
    # print(offsets)

    start_pub = rospy.Publisher("/Force/start", Bool, queue_size=10)
    stop_pub = rospy.Publisher("/Force/stop", Bool, queue_size=10)

    offset_pub = rospy.Publisher("/Force/offsets", Float32MultiArray, queue_size=10)
    offset_msg = Float32MultiArray()
    offset_msg.data = [0.0, 0.0]

    offset_grasp_pub = rospy.Publisher("/Force/grasp_offsets", Float32MultiArray, queue_size=10)
    offset_grasp_msg = Float32MultiArray()
    offset_grasp_msg.data = [0.0, 0.0]
    # offset_grasp_msg.data = [0.0, -0.014]

    sucess=[]
    offset_grasp_pub.publish(offset_grasp_msg)
    offset_pub.publish(offset_msg)
    start_pub.publish(Bool(True))
    input("Press Enter to start...")
    stop_pub.publish(Bool(True))
    print("Evaluating Rollouts...")
    i = 0
    while i < len(offsets):
        print()
        offset_msg.data = offsets[i]
        # offset_msg.data = [0.0, 0.0]
        offset_grasp_pub.publish(offset_grasp_msg)
        offset_pub.publish(offset_msg)
        stop_pub.publish(Bool(True))
        input(f"Offset {i}({offsets[i]}): Press Enter to start...")
        
        start_pub.publish(Bool(True))
        outcome = input("Was the task successful? (y/n(e, redo, undo)): ")
        stop_pub.publish(Bool(True))
        
        if outcome == 'y':
            sucess.append(i)
            print(f"Successful: {len(sucess)}/{len(offsets)}")
        elif outcome == 'e':
            print("Exiting...")
            break
        elif outcome.lower() == 'redo':
            print("Redoing...")
            i -= 1
        elif outcome.lower() == 'undo':
            if len(sucess) > 0:
                sucess.pop()
                i -= 2
                print(f"Undoing: {len(sucess)}/{len(offsets)}")
            else:
                print("Nothing to undo")
        else: 
            print(f"Failure: {len(sucess)}/{len(offsets)}")
        i += 1
    
    stop_pub.publish(Bool(True))
    print(f"Overall: {len(sucess)}/{len(offsets)}")
    print(f"Successful: {sucess}")
    

        

        

if __name__ == '__main__':
    eval()
