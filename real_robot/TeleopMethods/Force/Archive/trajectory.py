import h5py
from bag_h5 import h5_tree
import numpy as np

if __name__ == "__main__":
    input_file = "./trajectory.h5"

    data_h5 = h5py.File(input_file, 'r')
    h5_tree(data_h5)
    for data in data_h5['robot0_robot1_forcetorque-state']:
        print(f"Min: {np.min(data)}, Max: {np.max(data)}")