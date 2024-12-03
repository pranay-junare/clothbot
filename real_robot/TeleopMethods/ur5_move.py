#! /bin/env python3

# Import Launch2 Script to Re-run launch.py for Initialisation
from launch2 import UR5
import numpy as np


# Define Arms
arms = ['Thunder', 'Lightning']

# Set Home Position for Arms
home_angles = {}
home_angles['Thunder'] = [-180, -130, 130, -180, -90, 0]
home_angles['Lightning'] = [-180, -50, -130, -0, 90, +0]


# Define a Function to Convert Angles into Radians
def convert_into_radians(angles):
    radians = [angle/180 * np.pi for angle in angles]
    return radians


# Define a Function to Update Poses for given Arm
def update_pose(pose, offset):
    
    # For every value in Pose
    for i in range(len(pose)):

        # Update Pose with that Corresponding Offset
        pose[i] += offset[i]

    # Return Pose
    return pose

    

# Define the Main Function
def main():
    
    # Create ur5 Object to Control Arms
    ur5 = UR5()

    # Come to Home Position
    print("Setting Home Position")
    for arm in arms:
        ur5.URs.moveJ(arm, (convert_into_radians(home_angles[arm]), 0.5, 0.5))

    # Initialise Dictionary to store Poses wrt Base for all Arms
    poses = {}
    for arm in arms:
        
        # Get the Poses of Arms wrt Base
        poses[arm] = ur5.URs.get_receive(arm).getActualTCPPose()
    
    
    offsets = {}
    offsets['Thunder'] = [-0.1, 0, 0, 0, 0, 0]
    offsets['Lightning'] = [0, 0.1, 0, 0, 0, 0]

    # Update Poses for all Arms
    for arm in arms:

        # Update Pose with Offsets
        poses[arm] = update_pose(poses[arm], offsets[arm])

        # Move Arms to Final Pose
        ur5.URs.moveL(arm, (poses[arm], 0.1, 0.1, 0))
    

# Invoke Main Function
if __name__ == '__main__':
    main()
