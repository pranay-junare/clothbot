# Import Necessary Libraries
import pyrealsense2 as rs
import numpy as np
import cv2


# Define a Function to Read Image from Top Camera
def read_image_from_camera():
    
    # Try Block
    try:

        # Initialize RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Enable color stream
        pipeline.start(config)

        # Wait for the first frame
        print("Reading Image from Camera")
        frames = pipeline.wait_for_frames()

        # Get the depth and color frames
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            print("Depth or Color frame is not available.")
            return

        # Convert the color frame to a numpy array (OpenCV format)
        color_image = np.asanyarray(color_frame.get_data())
        cv2.imshow("Image", color_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Stop the pipeline
        pipeline.stop()

        # Return the Color Image
        return color_image

    # Catch any Exceptions
    except Exception as e:
        print(f"An error occurred: {e}")