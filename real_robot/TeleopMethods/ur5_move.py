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


# Define a Function to Set Arms to Home Position
def come_home_position(ur5):
    
    # For every Arm
    for arm in arms:

        # Goto Home position
        ur5.URs.moveJ(arm, (convert_into_radians(home_angles[arm]), 0.5, 0.5))
        
        # Open Gripper
        ur5.URs.get_gripper(arm).set(3)


# Define a Function to Get Arms Poses wrt Base Frame
def get_arms_poses(ur5):
    
    # Initialise Empty Dictionary to store Poses
    poses = {}

    # For every Arm
    for arm in arms:
        
        # Get the Poses of Arms wrt Base
        poses[arm] = ur5.URs.get_receive(arm).getActualTCPPose()

    # Return Poses
    return poses


# Define a Function to Update Poses for given Arm
def update_pose(pose, offset):
    
    # For every value in Pose
    for i in range(len(pose)):

        # Update Pose with that Corresponding Offset
        pose[i] += offset[i]

    # Return Pose
    return pose


# Define a Function to Grasp at a given Location for all Arms
def arms_grasp(ur5, poses, offsets):

    # For every Arm
    for arm in arms:

        # Update Pose with Offsets
        poses[arm] = update_pose(poses[arm], offsets[arm])

        # Move Arms to Final Pose
        ur5.URs.moveL(arm, (poses[arm], 0.02, 0.05, 0))
        
        # Close Gripper
        ur5.URs.get_gripper(arm).set(255)


# Define a Function to Move Arm along X-axis
def move_along_x(ur5, arm, distance, direction):
    
    # Get the Current Pose
    pose = ur5.URs.get_receive(arm).getActualTCPPose()
    
    # Update the Pose according to Direction
    if direction == "down":
        pose[0] += (distance * 0.01)
    else:
        pose[0] -= (distance * 0.01)
    
    # Move the Arm
    ur5.URs.moveL(arm, (pose, 0.02, 0.05, 0))


# Define a Function to Move Arm along Y-axis
def move_along_y(ur5, arm, distance, direction):
    
    # Get the Current Pose
    pose = ur5.URs.get_receive(arm).getActualTCPPose()
    
    # Update the Pose according to Direction
    if direction == "left":
        pose[1] += (distance * 0.01)
    else:
        pose[1] -= (distance * 0.01)
    
    # Move the Arm
    ur5.URs.moveL(arm, (pose, 0.02, 0.05, 0))


# Define a Function to Move Arm along Z-axis
def move_along_z(ur5, arm, distance, direction):
    
    # Get the Current Pose
    pose = ur5.URs.get_receive(arm).getActualTCPPose()
    
    # Update the Pose according to Direction
    if direction == "front":
        pose[2] += (distance * 0.01)
    else:
        pose[2] -= (distance * 0.01)
    
    # Move the Arm
    ur5.URs.moveL(arm, (pose, 0.02, 0.05, 0))


# Define a Function to Move Arm Front
def move_front(ur5, arm, distance):
    
    # Move the Arm along Z-axis
    move_along_z(ur5, arm, distance, "front")


# Define a Function to Move Arm Back
def move_back(ur5, arm, distance):
    
    # Move the Arm along Z-axis
    move_along_z(ur5, arm, distance, "back")


# Define a Function to Move Arm Left
def move_left(ur5, arm, distance):
    
    # Move the Arm along Y-axis
    if arm == "Lightning":
        distance = -distance
    move_along_y(ur5, arm, distance, "left")


# Define a Function to Move Arm Right
def move_right(ur5, arm, distance):
    
    # Move the Arm along Y-axis
    if arm == "Lightning":
        distance = -distance
    move_along_y(ur5, arm, distance, "right")


# Define a Function to Move Arm Up
def move_up(ur5, arm, distance):
    
    # Move the Arm along X-axis
    if arm == "Lightning":
        distance = -distance
    move_along_x(ur5, arm, distance, "up")


# Define a Function to Move Arm Down
def move_down(ur5, arm, distance):
    
    # Move the Arm along X-axis
    if arm == "Lightning":
        distance = -distance
    move_along_x(ur5, arm, distance, "down")

  

# Define the Main Function
def main():
    
    # Create ur5 Object to Control Arms
    ur5 = UR5()

    # Come to Home Position
    #print("Setting Home Position")
    #come_home_position(ur5)
    move_up(ur5, 'Thunder', 2)
    move_up(ur5, 'Lightning', 2)
    move_down(ur5, 'Thunder', 2)
    move_down(ur5, 'Lightning', 2)

    move_left(ur5, 'Thunder', 2)
    move_left(ur5, 'Lightning', 2)
    move_right(ur5, 'Thunder', 2)
    move_right(ur5, 'Lightning', 2)

    move_front(ur5, 'Thunder', 2)
    move_front(ur5, 'Lightning', 2)
    move_back(ur5, 'Thunder', 2)
    move_back(ur5, 'Lightning', 2)
    



# Invoke Main Function
if __name__ == '__main__':
    main()
