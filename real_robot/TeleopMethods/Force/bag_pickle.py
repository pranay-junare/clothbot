import rosbag
import sys
import os
import pickle

def bag_pickle(input, output):
    trajectory = []
    time = None
    for topic, msg, t in rosbag.Bag(input).read_messages():
        if time is None:    time = t
        if topic == "/thunder_cartesian_eef":
            delta = (t - time).to_sec()
            trajectory.append((delta, list(msg.data)))
    with open(output, 'wb') as f:
        pickle.dump(trajectory, f)

if __name__ == '__main__':
    # First argument is the input bag file dir and second argument is the output h5 file dir
    input_dir = sys.argv[1]
    output_dir = sys.argv[2]

    for file in os.listdir(input_dir):
        if file.endswith(".bag"):
            print(f"Processing {input_dir}/{file}")
            bag_pickle(f"{input_dir}/{file}", f"{output_dir}/{file}.pickle")
            print(f"Output: {output_dir}/{file}.pickle")
