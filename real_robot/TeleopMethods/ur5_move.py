#! /bin/env python3

# Import Launch2 Script to Re-run launch.py for Initialisation
from launch2 import UR5
import numpy as np
import time
import math


# Define Arms
arms = ['Thunder', 'Lightning']

# Set Home Position for Arms
home_angles = {}
home_angles['Thunder'] = [-180, -130, 130, -180, -90, 0]
home_angles['Lightning'] = [-180, -50, -130, -0, 90, +0]

# Set Custom Home Position for Arms
custom_home_angles = {}
custom_home_angles['Thunder'] = [-160.42516460340738, -129.96195150672278, 76.70328756889732, -179.97453638328042, -90.01844082751371, -0.009524926168668738]
custom_home_angles['Lightning'] = [-196.7935723486508, -49.557770332283646, -74.15220827143189, 0.025513249276600564, 89.99978393842473, 0.003288053024755972]


# Define Parameters to Control Arms
ARM_JOINT_ANGULAR_VELOCITY = 0.5
ARM_LINEAR_ACCELERATION = 0.1
ARM_LINEAR_VELOCITY = 0.5
ARM_LINEAR_WAIT_TIME_FACTOR = 0.1
ARM_ANGULAR_WAIT_TIME_FACTOR = 0.23
STEP_DISTANCE = 0.01


# Define a Function to Wait for given Time
def wait_time(distance, speed, time_factor):
    
    # Get the Time to Wait and Wait
    time_to_wait = (distance/speed) * time_factor
    time.sleep(time_to_wait)


# Define a Function to Convert Angles into Radians
def convert_into_radians(angles):
    
    # Intialise Empty List to store Radians
    radians = []

    # For every Angle
    for i in range(len(angles)):
        radian = angles[i] * np.pi / 180
        radians.append(radian)
    
    # Return Radians
    return radians


# Define a Function to Convert Radians into Angles
def convert_into_angles(radians):
    
    # Intialise Empty List to store Angles
    angles = []

    # For every Radian
    for i in range(len(radians)):
        angle = radians[i] * 180 / np.pi
        angles.append(angle)
    
    # Return Radians
    return angles


# Define a Function to Set Arms to Home Position
def come_home_position(ur5):
    
    # For every Arm
    print("Setting Home Position")
    for arm in arms:

        # Goto Home position
        ur5.URs.moveJ(arm, (convert_into_radians(home_angles[arm]), ARM_JOINT_ANGULAR_VELOCITY, ARM_JOINT_ANGULAR_VELOCITY))

    # Open Gripper
    ur5.URs.get_gripper(arm).set(3)
    
    # Wait 5 seconds for Arms to Reach Home Position
    time.sleep(5)


# Define a Function to Set Arms to Custom Home Position
def come_custom_home_position(ur5):
    
    # For every Arm
    print("Setting Custom Home Position")
    for arm in arms:

        # Goto Custom Home position
        ur5.URs.moveJ(arm, (convert_into_radians(custom_home_angles[arm]), ARM_JOINT_ANGULAR_VELOCITY, ARM_JOINT_ANGULAR_VELOCITY))

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
def update_pose(pose, offset, op):
    
    # If Operation is Addition
    if op == 'add':
        new_pose = np.add(pose, offset)
    else:
        new_pose = np.subtract(pose, offset)

    # Return Pose
    return new_pose


# Define a Function to Grasp for all Arms
def arms_grasp(ur5):

    # For every Arm
    for arm in arms:
        
        # Close Gripper
        ur5.URs.get_gripper(arm).set(255)
    
    # Wait for 2 seconds
    time.sleep(2)


# Define a Function to Unrasp for all Arms
def arms_ungrasp(ur5):

    # For every Arm
    for arm in arms:
        
        # Close Gripper
        ur5.URs.get_gripper(arm).set(5)


# Define a Function to Move Arm along X-axis for given Distance
def move_along_x(ur5, arm, distance, direction, arm_linear_acceleration, arm_linear_velocity):
    
    # Get the Current Pose
    pose = ur5.URs.get_receive(arm).getActualTCPPose()
    
    # Update the Pose according to Direction
    if direction == "down":
        pose[0] += (distance * STEP_DISTANCE)
    else:
        pose[0] -= (distance * STEP_DISTANCE)
    
    # Move the Arm
    ur5.URs.moveL(arm, (pose, arm_linear_acceleration, arm_linear_velocity))


# Define a Function to Move Arm along Y-axis for given Distance
def move_along_y(ur5, arm, distance, direction, arm_linear_acceleration, arm_linear_velocity):
    
    # Get the Current Pose
    pose = ur5.URs.get_receive(arm).getActualTCPPose()
    
    # Update the Pose according to Direction
    if direction == "left":
        pose[1] += (distance * STEP_DISTANCE)
    else:
        pose[1] -= (distance * STEP_DISTANCE)
    
    # Move the Arm
    ur5.URs.moveL(arm, (pose, arm_linear_acceleration, arm_linear_velocity))


# Define a Function to Move Arm along Z-axis for given Distance
def move_along_z(ur5, arm, distance, direction, arm_linear_acceleration, arm_linear_velocity):
    
    # Get the Current Pose
    pose = ur5.URs.get_receive(arm).getActualTCPPose()
    
    # Update the Pose according to Direction
    if direction == "front":
        pose[2] += (distance * STEP_DISTANCE)
    else:
        pose[2] -= (distance * STEP_DISTANCE)
    
    # Move the Arm
    ur5.URs.moveL(arm, (pose, arm_linear_acceleration, arm_linear_velocity))


# Define a Function to Move Arm Front for given Distance
def move_front(ur5, arm, distance, arm_linear_acceleration, arm_linear_velocity):
    
    # Move the Arm along Z-axis
    move_along_z(ur5, arm, distance, "front", arm_linear_acceleration, arm_linear_velocity)


# Define a Function to Move Arm Back for given Distance
def move_back(ur5, arm, distance, arm_linear_acceleration, arm_linear_velocity):
    
    # Move the Arm along Z-axis
    move_along_z(ur5, arm, distance, "back", arm_linear_acceleration, arm_linear_velocity)


# Define a Function to Move Arm Left for given Distance
def move_left(ur5, arm, distance, arm_linear_acceleration, arm_linear_velocity):
    
    # Move the Arm along Y-axis
    if arm == "Lightning":
        distance = -distance
    move_along_y(ur5, arm, distance, "left", arm_linear_acceleration, arm_linear_velocity)


# Define a Function to Move Arm Right for given Distance
def move_right(ur5, arm, distance, arm_linear_acceleration, arm_linear_velocity):
    
    # Move the Arm along Y-axis
    if arm == "Lightning":
        distance = -distance
    move_along_y(ur5, arm, distance, "right", arm_linear_acceleration, arm_linear_velocity)


# Define a Function to Move Arm Up for given Distance
def move_up(ur5, arm, distance, arm_linear_acceleration, arm_linear_velocity):
    
    # Move the Arm along X-axis
    if arm == "Lightning":
        distance = -distance
    move_along_x(ur5, arm, distance, "up", arm_linear_acceleration, arm_linear_velocity)


# Define a Function to Move Arm Down for given Distance
def move_down(ur5, arm, distance, arm_linear_acceleration, arm_linear_velocity): 
    
    # Move the Arm along X-axis
    if arm == "Lightning":
        distance = -distance
    move_along_x(ur5, arm, distance, "down", arm_linear_acceleration, arm_linear_velocity)


# Define a Function to Move Arm Back and Down for given Distance
def move_back_and_down(ur5, arm, distance, arm_linear_acceleration, arm_linear_velocity):

    # Get the Current Pose
    pose = ur5.URs.get_receive(arm).getActualTCPPose()

    # Update Pose to go Back
    pose[2] -= (distance * STEP_DISTANCE)

    # Update Distance according to Arm
    if arm == "Lightning":
        distance = -distance
    
    # Update Pose to go Down
    pose[0] += (distance * STEP_DISTANCE)

    # Move Arm Diagonally Back and Down
    ur5.URs.moveL(arm, (pose, arm_linear_acceleration, arm_linear_velocity))


# Define a Function to Move Arms Down and Grasp
def move_arms_down_and_grasp(ur5, distance, arm_linear_acceleration, arm_linear_velocity):

    # Move Arms Down and Grasp
    print("Moving Arms Down to Grasp")
    for arm in arms:
        move_down(ur5, arm, distance, arm_linear_acceleration, arm_linear_velocity)
    wait_time(distance, arm_linear_velocity, ARM_LINEAR_WAIT_TIME_FACTOR)
    arms_grasp(ur5)


# Define a Function to Move Arms Up to Lift
def move_arms_up_to_lift(ur5, distance, arm_linear_acceleration, arm_linear_velocity):

    # Move Arms Up to Lift
    print("Moving Arms Up to Lift")
    for arm in arms:
        move_up(ur5, arm, distance, arm_linear_acceleration, arm_linear_velocity)  
    wait_time(distance, arm_linear_velocity, ARM_LINEAR_WAIT_TIME_FACTOR)


# Define a Function to Get Joint Angles
def get_joint_angles(ur5):

    # Initialise Empty Dictionary to store Joint Angles
    joint_angles_rad = {}
    joint_angles_ang = {}

    # For every Arm
    for arm in arms:

        # Read the Joint Angles and Convert them into Degrees
        joint_angles_rad[arm] = ur5.URs.get_Q(arm)
        joint_angles_ang[arm] = convert_into_angles(joint_angles_rad[arm])
    
    # Return Joint Angles
    return joint_angles_ang


# Define a Function to Perform Fling Action 1
def fling_action_1(ur5, joint_angles, angle, arm_joint_angular_velocity):

    # Update Joint Angles
    joint_angles['Thunder'][2] -= angle
    joint_angles['Lightning'][2] += angle

    # For every Arm
    for arm in arms:

        # Perform Fling Action 1
        ur5.URs.moveJ(arm, (convert_into_radians(joint_angles[arm]), arm_joint_angular_velocity, arm_joint_angular_velocity))
    
    # Wait to complete the action
    wait_time(angle, arm_joint_angular_velocity, ARM_ANGULAR_WAIT_TIME_FACTOR)


# Define a Function to Move both Arms Front
def move_both_arms_front(ur5, distance, arm_linear_acceleration, arm_linear_velocity):

    # Move Both Arms Front
    for arm in arms:
        move_front(ur5, arm, distance, arm_linear_acceleration, arm_linear_velocity)

    # Wait to complete the action
    wait_time(distance, arm_linear_velocity, ARM_LINEAR_WAIT_TIME_FACTOR)


# Define a Function to Perform Fling Action 2
def fling_action_2(ur5, joint_angles, angle, arm_joint_angular_velocity):

    # Update Joint Angles
    joint_angles['Thunder'][2] += angle
    joint_angles['Lightning'][2] -= angle

    # For every Arm
    for arm in arms:

        # Perform Fling Action 1
        ur5.URs.moveJ(arm, (convert_into_radians(joint_angles[arm]), arm_joint_angular_velocity, arm_joint_angular_velocity))
    
    # Wait to complete the action
    wait_time(angle, arm_joint_angular_velocity, ARM_ANGULAR_WAIT_TIME_FACTOR)


# Define a Function to Perform Fling Action 3
def fling_action_3(ur5, distance, arm_linear_acceleration, arm_linear_velocity):

    # For every Arm
    for arm in arms:

        # Move Arms Back and Down simultaneously
        move_back_and_down(ur5, arm, distance, arm_linear_acceleration, arm_linear_velocity)
    
    # Wait to complete the action
    wait_time(distance * math.sqrt(2), arm_linear_velocity, ARM_LINEAR_WAIT_TIME_FACTOR)
    

# Define a Function to Fling Arms
def fling(ur5, swing, front, drag):
    
    # Perform Fling action 1
    print("Flinging the Cloth")
    fling_action_1(ur5, get_joint_angles(ur5), swing, arm_joint_angular_velocity = 3)

    # Move Arms Front
    move_both_arms_front(ur5, front, arm_linear_acceleration = 0.4, arm_linear_velocity = 3)
    time.sleep(1)
    
    # Perform Fling action 2
    fling_action_2(ur5, get_joint_angles(ur5), swing, arm_joint_angular_velocity = 3)

    # Perform Fling action 3
    fling_action_3(ur5, drag, arm_linear_acceleration = 0.2, arm_linear_velocity = 2)
    time.sleep(2)

    # Ungasp the Cloth
    arms_ungrasp(ur5)
    


# Define the Main Function
def main():

    # Create ur5 Object to Control Arms
    ur5 = UR5()
    
    # Come to Home Position
    come_home_position(ur5)

    # Come to Custom Home Position
    come_custom_home_position(ur5)

    '''

    # Move Both Arms down to Grasp Cloth
    move_arms_down_and_grasp(ur5, distance = 21, arm_linear_acceleration = 0.2, arm_linear_velocity = 0.5)
    
    # # Move Both Arms up to Lift Cloth
    move_arms_up_to_lift(ur5, distance = 30, arm_linear_acceleration = 0.2, arm_linear_velocity = 0.5)
    
    # Fling the Arms
    fling(ur5, swing = 15, front = 10, drag = 25)
    '''


# Invoke Main Function
if __name__ == '__main__':
    main()
