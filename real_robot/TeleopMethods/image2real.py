import pyrealsense2 as rs
import numpy as np
import cv2

def pixel_to_real_world_and_annotate(u, v):
    try:
        # Initialize RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Enable color stream
        pipeline.start(config)

        # Wait for the first frame
        frames = pipeline.wait_for_frames()

        # Get the depth and color frames
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            print("Depth or Color frame is not available.")
            return

        # Get the intrinsic parameters of the depth camera
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        fx = depth_intrinsics.fx
        fy = depth_intrinsics.fy
        cx = depth_intrinsics.ppx  # Principal point x (optical center)
        cy = depth_intrinsics.ppy  # Principal point y (optical center)

        print(f"Camera Intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}")

        # Get the depth at pixel (u, v)
        depth = depth_frame.get_distance(u, v)
        if depth == 0:
            print(f"No valid depth at pixel ({u}, {v}).")
            return

        print(f"Depth at pixel ({u}, {v}): {depth} meters")

        # Convert to real-world coordinates (X, Y, Z)
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth

        print(f"Real-world coordinates: X={X:.2f}, Y={Y:.2f}, Z={Z:.2f} meters")

        # Convert the color frame to a numpy array (OpenCV format)
        color_image = np.asanyarray(color_frame.get_data())

        # Annotate the point (u, v) on the color image
        # Draw a red circle at the specified (u, v)
        cv2.circle(color_image, (u, v), 5, (0, 0, 255), -1)  # Red circle
        cv2.putText(color_image, f"Point ({u}, {v})", (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Annotate the origin (cx, cy) on the color image
        # Draw a green circle at the origin (optical center)
        cv2.circle(color_image, (int(cx), int(cy)), 10, (0, 255, 0), -1)  # Green circle for the origin
        cv2.putText(color_image, "Origin", (int(cx) + 10, int(cy) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the image with the annotated point and origin
        cv2.imshow("Annotated Image", color_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Stop the pipeline
        pipeline.stop()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # Example pixel coordinates (u, v) in image space
    u = 320  # x-coordinate in image space (center of 640x480 image)
    v = 240  # y-coordinate in image space (center of 640x480 image)

    pixel_to_real_world_and_annotate(u, v)

