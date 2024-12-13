#! /bin/env python3
import ur5_move
import coordinate_transforms
from launch2 import UR5
import time


# Define the Main Function
def main():

    # Create ur5 Object to Control Arms
    ur5 = UR5()

    # Come to Custom Home Position and Ungrasp
    ur5_move.come_custom_home_position(ur5)
    ur5_move.arms_ungrasp(ur5)

    # Get the Pixels Coordinates on Image where to Grasp
    pixel_coordinates = {}
    pixel_coordinates['Lightning'] = [216, 168]
    pixel_coordinates['Thunder'] = [396, 162]

    # Come back to Actual Home Position
    ur5_move.come_home_position(ur5)

    # Move Arms to those Pixel Coordinates to Grasp
    coordinate_transforms.move_arms_to_pixel_coordinates(ur5, pixel_coordinates)

    # Move Both Arms up to Lift Cloth
    ur5_move.move_arms_up_to_lift(ur5, distance = 30, arm_linear_acceleration = 0.2, arm_linear_velocity = 0.5)
    
    # Fling the Arms
    ur5_move.fling(ur5, swing = 15, front = 10, drag = 25)

    # Go back to Actual Home Position
    ur5_move.come_home_position(ur5)
    

# Invoke Main Function
if __name__ == '__main__':
    main()