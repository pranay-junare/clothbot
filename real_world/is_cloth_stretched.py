import pyrealsense2 as rs
import numpy as np
import cv2
from scipy import ndimage
import matplotlib.pyplot as plt


def plt_batch(images, title="Debug View"):
    """Helper function to display debug images."""
    fig, axes = plt.subplots(1, len(images), figsize=(15, 5))
    fig.suptitle(title)
    for ax, (image, name) in zip(axes, images):
        ax.imshow(image, cmap='gray' if len(image.shape) == 2 else None)
        ax.set_title(name)
        ax.axis('off')
    plt.show()


def get_largest_component(binary_image):
    """Returns the largest connected component in a binary mask."""
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary_image, connectivity=8)
    if num_labels <= 1:
        return np.zeros_like(binary_image, dtype=np.uint8)  # No components found
    
    # Exclude the background label (0) and find the largest component
    largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    largest_component = (labels == largest_label).astype(np.uint8)
    return largest_component


def is_cloth_stretched(
        pipeline, 
        angle_tolerance=20, 
        threshold=20, 
        debug=False):
    """
    Determines if a cloth is stretched using live images from an Intel RealSense camera.

    Args:
        pipeline (rs.pipeline): Initialized RealSense pipeline for capturing frames.
        angle_tolerance (float): Maximum allowable angle to consider the cloth stretched.
        threshold (float): Minimum stretchedness threshold.
        debug (bool): If True, displays intermediate debug images.

    Returns:
        bool: True if the cloth is stretched, False otherwise.
    """
    FOREGROUND_BACKGROUND_DIST = 1000  # Example value, adjust as necessary
    GRIPPER_LINE = 240  # Example value, adjust based on the depth image resolution

    # Start streaming frames
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        print("Error: Could not retrieve frames from RealSense camera.")
        return False

    # Convert frames to numpy arrays
    rgb = np.asanyarray(color_frame.get_data())
    depth = np.asanyarray(depth_frame.get_data())

    imshows = []
    fgbg = np.logical_and(
        depth < FOREGROUND_BACKGROUND_DIST, depth != 0).astype(np.uint8)
    
    if debug:
        imshows = [(rgb, 'rgb'), (depth, 'depth'), (fgbg.copy(), 'fgbg')]

    fgbg = cv2.morphologyEx(
        fgbg, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (5, 5)),
        iterations=4)
    if debug:
        imshows.append((fgbg.copy(), 'mask'))

    gripper_strip = fgbg[GRIPPER_LINE, :]

    # Find grippers
    center = len(gripper_strip) // 2
    right_gripper_pix = center + 1
    while not gripper_strip[right_gripper_pix]:
        right_gripper_pix += 1
        if right_gripper_pix == len(gripper_strip) - 1:
            break
    left_gripper_pix = center - 1
    while not gripper_strip[left_gripper_pix]:
        left_gripper_pix -= 1
        if left_gripper_pix == 0:
            break
    center = int((left_gripper_pix + right_gripper_pix) / 2)
    fgbg[:, :left_gripper_pix] = 0
    fgbg[:, right_gripper_pix:] = 0
    fgbg[:GRIPPER_LINE, :] = 0

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (5, 5))
    line_mask = cv2.morphologyEx(
        fgbg.copy(), cv2.MORPH_CLOSE, kernel,
        iterations=4)

    kernel = np.array([[-1], [0], [1]] * 3)
    line_mask = cv2.filter2D(fgbg, -1, kernel)
    if debug:
        imshows.append((fgbg.copy(), 'filtered'))
        imshows.append((line_mask.copy(), 'horizontal edges'))

    line_mask = get_largest_component(
        cv2.morphologyEx(
            line_mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (10, 10)), iterations=5))
    if debug:
        imshows.append((line_mask.copy(), 'largest component'))

    # Find angle to rotate to
    points = np.array(np.where(line_mask.copy() == 1)).T
    points = np.array(sorted(points, key=lambda x: x[1]))
    max_x = points[-1][1]
    min_x = points[0][1]
    min_x_y = min(points[(points[:, 1] == min_x)],
                  key=lambda pnt: pnt[0])[0]
    max_x_y = min(points[(points[:, 1] == max_x)],
                  key=lambda pnt: pnt[0])[0]
    angle = 180 * np.arctan((max_x_y - min_x_y) / (max_x - min_x)) / np.pi
    line_mask = ndimage.rotate(line_mask, angle, reshape=False)
    if debug:
        print('angle:', angle)
        img = np.zeros(line_mask.shape).astype(np.uint8)
        img = cv2.circle(
            img=img,
            center=(min_x, min_x_y),
            radius=10, color=1, thickness=3)
        img = cv2.circle(
            img=img,
            center=(max_x, max_x_y),
            radius=10, color=1, thickness=3)
        imshows.append((img, 'circled'))
        imshows.append((line_mask.copy(), f'rotated ({angle:.02f}°)'))

    # Check if angle is too sharp, indicating the cloth is not stretched
    y_values = np.array(np.where(line_mask == 1))[0, :]
    min_coord = y_values.min()
    max_coord = y_values.max()
    stretchedness = 1 / ((max_coord - min_coord) / line_mask.shape[0])
    too_tilted = np.abs(angle) > angle_tolerance
    stretch = (not too_tilted) and (stretchedness > threshold)

    if debug:
        print(stretchedness)
        plt_batch(
            imshows, f'Stretchness: {stretchedness:.02f}, Stretched: {stretch}')

    return stretch


# Main RealSense Pipeline
def main():
    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    try:
        while True:
            stretched = is_cloth_stretched(pipeline, debug=True)
            print(f"Cloth is {'stretched' if stretched else 'not stretched'}.")

    except KeyboardInterrupt:
        print("Stopping pipeline...")

    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()

