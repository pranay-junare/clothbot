#! /bin/env python3

# Import Launch2 Script to Re-run launch.py for Initialisation
import ur5_move
import numpy as np
import time
import math


# Define Arms
arms = ['Thunder', 'Lightning']

# Define the Poses of Arms at Origin wrt Base
pose_at_origin_wrt_base = {}
pose_at_origin_wrt_base['Thunder'] = [0.4287960632960006, 0.3725351444832463, 0.5076178275754467, -1.2096211591369221, 1.2086797731034713, -1.2153842149816727]
pose_at_origin_wrt_base['Lightning'] = [-0.4289481861070213, 0.3726487357833324, 0.49861378937777756, -1.2036593360758385, -1.211274018116346, 1.2073764623980814]

# Define the Origin wrt Camera frame
origin_wrt_camera = [0.05, -0.03, 1.2]
camera_origin = [320, 240]

# Define the Camera Intrinsics
camera_intrinsics = {'fx': 461.16796875, 'fy': 460.5, 'cx': 299.328125, 'cy': 251.37109375}
depth = 1.2


# Define a Function to Convert Camera coordinates to real-world coordinates (X, Y)
def camera_to_world(pixel_coordinates):

    # Get the Coordinates
    u, v = pixel_coordinates

    # Calculate the Real-world Coordinates
    X = round((u - camera_intrinsics['cx']) * depth / camera_intrinsics['fx'], 2)
    Y = round((v - camera_intrinsics['cy']) * depth / camera_intrinsics['fy'], 2)

    # Return the 3D World Coordinates
    return [X, Y, depth]


# Define a Function to get the Arms Position wrt to Real world Origin
def get_arms_positions_wrt_origin(ur5):

    # Initialise Empty Dictionary to store Arm Position wrt Origin
    arm_position_wrt_origin = {}

    # For every Arm
    for arm in arms:

        # Store the Arm Position wrt Origin
        arm_position_wrt_origin[arm] = ur5_move.update_pose(get_arms_poses(ur5)[arm], pose_at_origin_wrt_base[arm], op = "diff")[:3]
    
        # Store Position Temporarily
        temp = arm_position_wrt_origin[arm]

        # Swap Axis from Base link from to Camera frame
        arm_position_wrt_origin[arm] = [-temp[1], -temp[2], temp[0]]
        
    # Return World Position
    return arm_position_wrt_origin


# Define a Function to get World Position wrt to Camera
def get_world_position_wrt_camera(ur5):

    # Initialise Empty Dictionary to store World Position wrt Camera
    world_position_wrt_camera = {}
    world_position_wrt_origin = {}

    # For every Arm
    for arm in arms:

        # Store the World Position of Arm wrt Origin
        world_position_wrt_origin[arm] = update_pose(get_arms_poses(ur5)[arm], pose_at_origin_wrt_base[arm], op = "diff")
        if arm == "Lightning":
            world_position_wrt_origin[arm][0] *= -1
            world_position_wrt_origin[arm][1] *= -1

        # Do Coordinate Transformation
        temp = world_position_wrt_origin[arm]
        world_position_wrt_origin[arm] = [-temp[1], -temp[2], temp[0]]

        # Add Origin wrt Camera to get World Position wrt Camera
        world_position_wrt_camera[arm] = update_pose(world_position_wrt_origin[arm], origin_wrt_camera, op = "add")
    
    # Return World Position
    return world_position_wrt_camera


# Define a Function to move Arms to Pixel Coordinates
def move_arms_to_pixel_coordinates(ur5, pixel_coordinates):

    # Get the Required World Position wrt Camera
    required_world_position_wrt_camera = camera_to_world(pixel_coordinates)

    # Get the Required World Position wrt Origin
    required_world_position_wrt_origin = update_pose(required_world_position_wrt_camera, origin_wrt_camera, op = "diff")
    
    # Get the Arms Position wrt Origin
    arms_position_wrt_origin = get_arms_positions_wrt_origin(ur5)

    # Get the Required Offset
    offset = update_pose(required_world_position_wrt_origin, arms_position_wrt_origin['Thunder'], op = "diff")
    
    h1 = get_world_position_wrt_camera(ur5)['Thunder'][2]
    h = depth - h1

    print(offset, h)