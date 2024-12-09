#! /bin/env python3

# Import Launch2 Script to Re-run launch.py for Initialisation
from launch2 import UR5
import numpy as np
import time


# Define Arms
arms = ['Thunder', 'Lightning']

# Set Home Position for Arms
home_angles = {}
#home_angles['Thunder'] = [-180, -130, 130, -180, -90, 0]
#home_angles['Lightning'] = [-180, -50, -130, -0, 90, +0]
home_angles['Thunder'] = [-180, -130, 130, -180, -90, 0]
home_angles['Lightning'] = [-180, -50, -130, -0, 90, +0]

# Define Parameters to Control Arms
ARM_JOINT_VELOCITY = 0.5
ARM_ACCELERATION = 0.1
ARM_VELOCITY = 0.5
ARM_STEP = 0.01
ARM_WAIT_TIME_FACTOR_LINEAR = 3
ARM_WAIT_TIME_FACTOR_ANGULAR = 14


# Define a Function to Convert Angles into Radians
def convert_into_radians(angles):
    radians = [angle/180 * np.pi for angle in angles]
    return radians


# Define a Function to Convert Radians into Angles
def convert_into_angles(radians):
    angles = [radian/np.pi * 180 for radian in radians]
    return angles


# Define a Function to Set Arms to Home Position
def come_home_position(ur5):
    
    # For every Arm
    print("Setting Home Position")
    for arm in arms:

        # Goto Home position
        ur5.URs.moveJ(arm, (convert_into_radians(home_angles[arm]), ARM_JOINT_VELOCITY, ARM_JOINT_VELOCITY))

    # Open Gripper
    ur5.URs.get_gripper(arm).set(3)
    
    # Wait 5 seconds for Arms to Reach Home Position
    time.sleep(5)


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


# Define a Function to Grasp for all Arms
def arms_grasp(ur5):

    # For every Arm
    for arm in arms:
        
        # Close Gripper
        ur5.URs.get_gripper(arm).set(255)
    time.sleep(2)


# Define a Function to Unrasp for all Arms
def arms_ungrasp(ur5):

    # For every Arm
    for arm in arms:
        
        # Close Gripper
        ur5.URs.get_gripper(arm).set(5)


# Define a Function to Move Arm along X-axis
def move_along_x(ur5, arm, distance, direction):
    
    # Get the Current Pose
    pose = ur5.URs.get_receive(arm).getActualTCPPose()
    
    # Update the Pose according to Direction
    if direction == "down":
        pose[0] += (distance * ARM_STEP)
    else:
        pose[0] -= (distance * ARM_STEP)
    
    # Move the Arm
    ur5.URs.moveL(arm, (pose, ARM_ACCELERATION, ARM_VELOCITY))


# Define a Function to Move Arm along Y-axis
def move_along_y(ur5, arm, distance, direction):
    
    # Get the Current Pose
    pose = ur5.URs.get_receive(arm).getActualTCPPose()
    
    # Update the Pose according to Direction
    if direction == "left":
        pose[1] += (distance * ARM_STEP)
    else:
        pose[1] -= (distance * ARM_STEP)
    
    # Move the Arm
    ur5.URs.moveL(arm, (pose, ARM_ACCELERATION, ARM_VELOCITY))


# Define a Function to Move Arm along Z-axis
def move_along_z(ur5, arm, distance, direction):
    
    # Get the Current Pose
    pose = ur5.URs.get_receive(arm).getActualTCPPose()
    
    # Update the Pose according to Direction
    if direction == "front":
        pose[2] += (distance * ARM_STEP)
    else:
        pose[2] -= (distance * ARM_STEP)
    
    # Move the Arm
    ur5.URs.moveL(arm, (pose, ARM_ACCELERATION, ARM_VELOCITY))


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


# Define a Function to Move Arm Back and Down
def move_back_and_down(ur5, arm, distance):

    # Get the Current Pose
    pose = ur5.URs.get_receive(arm).getActualTCPPose()

    # Update Pose to go Back
    pose[2] -= (distance * ARM_STEP)

    # Update Distance according to Arm
    if arm == "Lightning":
        distance = -distance
    
    # Update Pose to go Down
    pose[0] += (distance * ARM_STEP)

    # Move Arm Diagonally Back and Down
    ur5.URs.moveL(arm, (pose, ARM_ACCELERATION, ARM_VELOCITY))



# Define a Function to Move Arms Down and Grasp
def move_arms_down_and_grasp(ur5, distance):

    # Move Arms Down and Grasp
    print("Moving Arms Down to Grasp")
    move_down(ur5, "Thunder", distance)
    move_down(ur5, "Lightning", distance)
    time.sleep(distance/ARM_WAIT_TIME_FACTOR_LINEAR)
    arms_grasp(ur5)


# Define a Function to Move Arms Up to Lift
def move_arms_up_to_lift(ur5, distance):

    # Move Arms Up to Lift
    print("Moving Arms Up to Lift")
    move_up(ur5, "Thunder", distance)
    move_up(ur5, "Lightning", distance)    
    time.sleep(distance/ARM_WAIT_TIME_FACTOR_LINEAR)


# Define a Function to Get Joint Angles
def get_joint_angles(ur5):

    # Initialise Empty Dictionary to store Joint Angles
    joint_angles = {}

    # For every Arm
    for arm in arms:

        # Read the Joint Angles and Convert them into Degrees
        joint_angles[arm] = ur5.URs.get_Q(arm)
        joint_angles[arm] = convert_into_angles(joint_angles[arm])
    
    # Return Joint Angles
    return joint_angles


# Define a Function to Perform Fling Action 1
def fling_action_1(ur5, joint_angles, angle):

    # Update Joint Angles
    joint_angles['Thunder'][2] -= angle
    joint_angles['Lightning'][2] += angle

    # For every Arm
    for arm in arms:

        # Perform Fling Action 1
        ur5.URs.moveJ(arm, (convert_into_radians(joint_angles[arm]), 1.5, 1.5))
    
    # Wait to complete the action
    time.sleep(angle/ARM_WAIT_TIME_FACTOR_ANGULAR)


# Define a Function to Move both Arms Front
def move_both_arms_front(ur5, distance):

    # Move Both Arms Front
    move_front(ur5, "Thunder", distance)
    move_front(ur5, "Lightning", distance)

    # Wait to complete the action
    time.sleep(distance/ARM_WAIT_TIME_FACTOR_LINEAR)


# Define a Function to Perform Fling Action 2
def fling_action_2(ur5, joint_angles, angle):

    # Update Joint Angles
    joint_angles['Thunder'][2] += angle
    joint_angles['Lightning'][2] -= angle

    # For every Arm
    for arm in arms:

        # Perform Fling Action 1
        ur5.URs.moveJ(arm, (convert_into_radians(joint_angles[arm]), 1.5, 1.5))
    
    # Wait to complete the action
    time.sleep(angle/ARM_WAIT_TIME_FACTOR_ANGULAR)


# Define a Function to Perform Fling Action 3
def fling_action_3(ur5, distance):

    # For every Arm
    for arm in arms:

        # Move Arms Back and Down simultaneously
        move_back_and_down(ur5, arm, distance)
    
    # Wait to complete the action
    time.sleep(distance/ARM_WAIT_TIME_FACTOR_LINEAR)
    

# Define a Function to Fling Arms
def fling(ur5):

    print("Flinging the Cloth")

    # Get the Joint Angles for Arms
    joint_angles = get_joint_angles(ur5)
    
    # Perform Fling action 1
    fling_action_1(ur5, joint_angles, angle = 15)

    # Move Arms Front
    move_both_arms_front(ur5, distance = 15)

    # Get the Joint Angles for Arms
    joint_angles = get_joint_angles(ur5)
    
    # Perform Fling action 2
    fling_action_2(ur5, joint_angles, angle = 15)

    # Perform Fling action 3
    fling_action_3(ur5, distance = 25)

    # Ungasp the Cloth
    arms_ungrasp(ur5)
    


# Define the Main Function
def main():
    
    # Create ur5 Object to Control Arms
    ur5 = UR5()

    # Come to Home Position
    come_home_position(ur5)

    # Move Both Arms down to Grasp Cloth
    move_arms_down_and_grasp(ur5, distance = 21)
    
    # # Move Both Arms up to Lift Cloth
    move_arms_up_to_lift(ur5, distance = 30)
    
    # Fling the Arms
    fling(ur5)
    
    
    

# Invoke Main Function
if __name__ == '__main__':
    main()
