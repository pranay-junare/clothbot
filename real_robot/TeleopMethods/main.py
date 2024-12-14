#! /bin/env python3
import read_images
import ur5_move
import coordinate_transforms
from launch2 import UR5
import time
import cv2


# Define the Main Function
def main():

    # Create ur5 Object to Control Arms
    ur5 = UR5()

    # Come to Custom Home Position and Ungrasp
    ur5_move.come_custom_home_position(ur5)
    ur5_move.arms_ungrasp(ur5)

    # Read Image from Top Camera
    image = read_images.read_image_from_camera()
    cv2.imwrite("TeleopMethods/Top_image.jpg", image)

    '''
    # Get the Pixels Coordinates on Image where to Grasp
    pixel_coordinates = {}
    pixel_coordinates['Lightning'] = [216, 168]
    pixel_coordinates['Thunder'] = [396, 162]

    # Come back to Actual Home Position
    ur5_move.come_home_position(ur5)

    # Move Arms to those Pixel Coordinates to Grasp
    print("Moving Arms to Pixel Coordinates to Grasp")
    coordinate_transforms.move_arms_to_pixel_coordinates(ur5, pixel_coordinates)

    # Come back to Actual Home Position
    ur5_move.come_home_position(ur5, ungrasp = False)
    
    # Fling the Arms
    ur5_move.fling(ur5, swing = 40, drag = 25)

    '''

    # Go back to Actual Home Position
    ur5_move.come_home_position(ur5)
    
    

# Invoke Main Function
if __name__ == '__main__':
    main()