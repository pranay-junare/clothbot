#! /bin/env python3

# Import Launch2 Script to Re-run launch.py for Initialisation
from launch2 import UR5
import ur5_move
import numpy as np
import time
import math


# Define Arms
arms = ['Thunder', 'Lightning']

# Define the Poses of Arms at Origin wrt Base Frame
pose_at_origin_wrt_base_frame = {}
pose_at_origin_wrt_base_frame['Thunder'] = [0.4187960632960006, 0.3725351444832463, 0.5076178275754467, -1.2096211591369221, 1.2086797731034713, -1.2153842149816727]
pose_at_origin_wrt_base_frame['Lightning'] = [-0.4189481861070213, 0.3726487357833324, 0.49861378937777756, -1.2036593360758385, -1.211274018116346, 1.2073764623980814]

# Define the Origin wrt Camera Frame
camera_origin_wrt_camera_frame = [0.05, -0.03, 1.2]

# Define the Camera Intrinsics Parameters
camera_intrinsics = {'fx': 461.16796875, 'fy': 460.5, 'cx': 299.328125, 'cy': 251.37109375, 'depth': 1.2}


# Define a Function to Convert Coordinates from Camera Frame to World Frame
def convert_camera_frame_to_world_frame(coordinates_wrt_camera_frame):

    # Store Coordinates Temporarily
    temp = coordinates_wrt_camera_frame

    # Update the Coordinates wrt to World Frame
    coordinates_wrt_world_frame = [-temp[0], temp[1], -temp[2]]

    # Return the Coordinates wrt World Frame
    return coordinates_wrt_world_frame


# Define a Function to Convert Coordinates from Camera Frame to World Frame
def camera_frame_to_world_frame(pixel_coordinates):

    # Initialise the Coordinates for Arms wrt World Frame
    coordinates_wrt_world_frame = {}

    # For every Arm
    for arm in arms:

        # Get the Coordinates
        u, v = pixel_coordinates[arm]

        # Calculate the Real-world Coordinates wrt Camera Frame
        x = round((u - camera_intrinsics['cx']) * camera_intrinsics['depth'] / camera_intrinsics['fx'], 2)
        y = round((v - camera_intrinsics['cy']) * camera_intrinsics['depth'] / camera_intrinsics['fy'], 2)
        z = camera_intrinsics['depth']

        # Get the Coordinates wrt Camera Frame
        coordinates_wrt_camera_frame = ur5_move.update_pose([x, y, z], camera_origin_wrt_camera_frame, op = 'diff')
        
        # Convert the Coordinates wrt Camera Frame to World Frame
        coordinates_wrt_world_frame[arm] = convert_camera_frame_to_world_frame(coordinates_wrt_camera_frame)

    # Return the Coordinates wrt World Frame
    return coordinates_wrt_world_frame


# Define a Function to Convert Coordinates from Base Frame to World Frame
def base_frame_to_world_frame(coordinates_wrt_base_frame):

    # Store Coordinates Temporarily
    temp = coordinates_wrt_base_frame

    # Update the Coordinates wrt to World Frame
    coordinates_wrt_world_frame = [temp[1], -temp[2], -temp[0]]

    # Return the Coordinates wrt World Frame
    return coordinates_wrt_world_frame


# Define a Function to Convert Coordinates from World Frame to Base Frame
def world_frame_to_base_frame(coordinates_wrt_world_frame):

    # Store Coordinates Temporarily
    temp = coordinates_wrt_world_frame

    # Update the Coordinates wrt to Base Frame
    coordinates_wrt_base_frame = [-temp[2], temp[0], -temp[1]]

    # Return the Coordinates wrt Base Frame
    return coordinates_wrt_base_frame


# Define a Function to Get the Position of Arms wrt World Frame
def get_arms_position_wrt_world_frame(ur5, arms):

    # Initialise the Position of Arms wrt World Frame
    arms_position_wrt_world_frame = {}

    # For all Arms
    for arm in arms:

        # Get the Position of Arm wrt Base Frame
        arm_position_wrt_base_frame = ur5.URs.get_receive(arm).getActualTCPPose()

        # Get the Position of Arm wrt Base Frame
        arm_position_from_origin_wrt_base_frame = ur5_move.update_pose(arm_position_wrt_base_frame, pose_at_origin_wrt_base_frame[arm], op = 'diff')

        # Convert the Arm Position from Origin wrt Base Frame to World Frame
        arms_position_wrt_world_frame[arm] = base_frame_to_world_frame(arm_position_from_origin_wrt_base_frame)
        if arm == "Lightning":
            arms_position_wrt_world_frame[arm][0] *= -1
            arms_position_wrt_world_frame[arm][2] *= -1
    
    # Return the Arms Position wrt World Frame
    return arms_position_wrt_world_frame
        

# Define a Function to Move the Arms to Pixel Coordinates
def move_arms_to_pixel_coordinates(ur5, pixel_coordinates):

    # Get the Final Position of Arms wrt World Frame
    final_arms_position_wrt_world_frame = camera_frame_to_world_frame(pixel_coordinates)

    # Get the Initial Position of Arms wrt World Frame
    arms_position_wrt_world_frame = get_arms_position_wrt_world_frame(ur5, arms)

    # Get the Displacement of Arms wrt Base Frame
    displacement_wrt_base_frame = {}
    for arm in arms:
        displacement_wrt_world_frame = ur5_move.update_pose(final_arms_position_wrt_world_frame[arm], arms_position_wrt_world_frame[arm], op = 'diff')
        displacement_wrt_base_frame[arm] = world_frame_to_base_frame(displacement_wrt_world_frame)
        if arm == "Lightning":
            displacement_wrt_base_frame[arm][0] *= -1
            displacement_wrt_base_frame[arm][1] *= -1
        for i in range(3):
            displacement_wrt_base_frame[arm].append(0)
    
    # Get the Current and Final Position of Arms wrt Base Frame
    current_arms_position_wrt_base_frame = {}
    final_arms_position_wrt_base_frame = {}
    for arm in arms:
        current_arms_position_wrt_base_frame[arm] = ur5.URs.get_receive(arm).getActualTCPPose()
        final_arms_position_wrt_base_frame[arm] = ur5_move.update_pose(current_arms_position_wrt_base_frame[arm], displacement_wrt_base_frame[arm], op = 'add')
    
    # Move the Arms to the Desired Position and Grasp
    final_arms_position_wrt_base_frame['Lightning'][1] += 0.1
    for arm in arms:
        ur5.URs.moveL(arm, (final_arms_position_wrt_base_frame[arm], 0.1, 0.1))
    time.sleep(7)
    ur5_move.arms_grasp(ur5)  