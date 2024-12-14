#! /bin/env python3
import coordinate_transforms
from launch2 import UR5
import ur5_move
import cv2


# Define a Global variable to store the Coordinates of the Points Clicked on the Image for both Arms
global pixel_coordinates
pixel_coordinates = {}


# Define a Function to Read Images from both Cameras
def read_images_from_cameras():

    # Read Image from Top camera and Front Camera
    top_camera_image = cv2.imread('TeleopMethods/top_image.jpg')
    front_camera_image = cv2.imread('TeleopMethods/front_image.jpg')

    # Return the Images
    return top_camera_image, front_camera_image


# Define a Function to display the coordinates of the points clicked on the image  
def click_event(event, x, y, flags, params): 
  
    # Check for Left mouse clicks 
    if event == cv2.EVENT_LBUTTONDOWN: 
        
        # Display the coordinates according to the arm
        if len(pixel_coordinates) == 0:
            print("First choose Coordinates for Lightning Arm")
            print(x, ' ', y)
            pixel_coordinates['Lightning'] = [x, y]
        elif len(pixel_coordinates) == 1:
            print("Now choose Coordinates for Thunder Arm")
            print(x, ' ', y)
            pixel_coordinates['Thunder'] = [x, y]
        else:
            return pixel_coordinates
        

# Define a Function to Choose Points on the Image to Grasp
def choose_points_on_image(top_camera_image):

    # Display the Image
    cv2.imshow('Top_Image', top_camera_image)

    # Set Mouse Callback
    cv2.setMouseCallback('Top_Image', click_event)

    # Wait for the User to Click on the Image
    cv2.waitKey(0)

    # Close the Image
    cv2.destroyAllWindows()

    # Return the Pixel Coordinates
    return pixel_coordinates


# Define a Function to Get the Distance to Stretch
def get_distance_to_stretch(front_camera_image):
    
    return 4


# Define the Main Function
def main():

    # Create ur5 Object to Control Arms
    ur5 = UR5()

    # Come to Custom Home Position and Ungrasp
    ur5_move.come_custom_home_position(ur5)
    ur5_move.arms_ungrasp(ur5)
 
    # Read Image from Top Camera
    top_camera_image, _ = read_images_from_cameras()

    # Choose 2 Points on the Image to Grasp
    pixel_coordinates = choose_points_on_image(top_camera_image)

    # Come back to Actual Home Position
    ur5_move.come_home_position(ur5)

    # Move Arms to those Pixel Coordinates to Grasp
    print("Moving Arms to Pixel Coordinates to Grasp")
    coordinate_transforms.move_arms_to_pixel_coordinates(ur5, pixel_coordinates)

    # Come back to Actual Home Position
    ur5_move.come_home_position(ur5, ungrasp = False)
    
    # Perform Fling action Part 1
    ur5_move.fling_1(ur5, swing = 40)

    # Read the Image from Front Camera to decide how much to stretch
    _, front_camera_image = read_images_from_cameras()
    stretch_distance = get_distance_to_stretch(front_camera_image)

    # Stretch Cloth
    ur5_move.stretch_cloth(ur5, stretch_distance, arm_linear_acceleration = 0.1, arm_linear_velocity = 1)

    # Perform Fling action Part 2
    ur5_move.fling_2(ur5, drag = 25)

    # Go back to Actual Home Position
    ur5_move.come_home_position(ur5)
    
    

# Invoke Main Function
if __name__ == '__main__':
    main()