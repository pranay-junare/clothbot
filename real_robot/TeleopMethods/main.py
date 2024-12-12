#! /bin/env python3
import ur5_move
from launch2 import UR5
import time




# Define the Main Function
def main():

    # Create ur5 Object to Control Arms
    ur5 = UR5()

    # Come to Home Position
    ur5_move.come_home_position(ur5)

    # Move the Arms to the required Pixel in Camera Frame
    ur5_move.move_arms_to_pixel_coordinates(ur5, [320, 240])
    

# Invoke Main Function
if __name__ == '__main__':
    main()