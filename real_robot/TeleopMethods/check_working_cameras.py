import pyrealsense2 as rs
import cv2
import numpy as np
HARDWARE_RESET = False

def stream_realsense_cameras():
    try:
        # Create a context object to manage connections to RealSense devices
        context = rs.context()


        # Query all connected RealSense devices
        devices = context.query_devices()

        if HARDWARE_RESET == True:
            for device in devices:
                device.hardware_reset()

        device_count = len(devices)

        if device_count == 0:
            print("No RealSense cameras detected.")
            return

        print(f"Number of RealSense cameras detected: {device_count}")

        # Initialize pipelines for each connected device
        pipelines = []
        for i, device in enumerate(devices):
            print(f"Initializing Camera {i + 1}:")
            print(f"  Name: {device.get_info(rs.camera_info.name)}")
            print(f"  Serial Number: {device.get_info(rs.camera_info.serial_number)}")

            # Create a pipeline for each camera
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(device.get_info(rs.camera_info.serial_number))
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipeline.start(config)
            pipelines.append(pipeline)

        # Stream from all cameras
        while True:
            frames = []
            for i, pipeline in enumerate(pipelines):
                # Wait for a new frame and get the color frame
                frameset = pipeline.wait_for_frames()
                color_frame = frameset.get_color_frame()
                if not color_frame:
                    continue

                # Convert RealSense frame to numpy array
                color_image = cv2.cvtColor(
                    np.asanyarray(color_frame.get_data()), cv2.COLOR_BGR2RGB
                )
                frames.append((f"Camera {i + 1}", color_image))

            # Display the frames
            for window_name, frame in frames:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imshow(window_name, frame)

            # Break the loop on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Stop all pipelines and close windows
        for pipeline in pipelines:
            pipeline.stop()
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    stream_realsense_cameras()

