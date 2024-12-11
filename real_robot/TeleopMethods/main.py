#! /bin/env python3

import image2real
import ur5_move
from launch2 import UR5
import time


# Define the Main Function
def main():

    # Create ur5 Object to Control Arms
    ur5 = UR5()

    # Come to Home Position
    ur5_move.come_home_position(ur5)

    # Get the World Position of Arms wrt Camera
    world_position_wrt_camera = ur5_move.get_world_position_wrt_camera(ur5)['Lightning']
    
    #a = image2real.pixel_to_real_world_and_annotate(150, 120)
    a = [-0.39, -0.34, 1.2]

    # Get the Pose of Arms
    pose = ur5_move.get_arms_poses(ur5)['Lightning']

    # Get the Displacement to go
    displacement = ur5_move.update_pose(a, world_position_wrt_camera, op = "diff")

    temp = displacement
    displacement = [temp[2], -temp[0], -temp[1], 0, 0, 0]
    print(displacement)

    pose = ur5_move.update_pose(displacement, pose, op = "add")
    ur5.URs.moveL('Lightning', (pose, 0.1, 0.1))
    

# Invoke Main Function
if __name__ == '__main__':
    main()