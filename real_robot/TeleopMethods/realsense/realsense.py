import pyrealsense2 as rs
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import sys

try:
    if len(sys.argv) < 2:
        camera = None
    else:
        camera = sys.argv[1]
    rospy.init_node(f"Camera", anonymous=True)

    cameras = {
        '/lightning/wrist/': {
            'serial': '126122270307', 
            'hz': 20,
            # 'hz': 5,
            'RGB': (480, 270, rs.format.rgb8, 30),
            # 'RGB': (480, 270, rs.format.rgb8, 
            # 15),
            'depth': None
        },
        '/thunder/wrist/': {
            'serial': '126122270722',
            'hz': 20,
            # 'hz': 5,
            'RGB': (480, 270, rs.format.rgb8, 30),
            # 'RGB': (480, 270, rs.format.rgb8, 6),
            'depth': None
        },
        '/both/front/': {
            'serial': 'f1371786',
            'hz': 5,
            # 'RGB': (1024, 768, rs.format.rgb8, 30),
            # 'depth': (1024, 768, rs.format.z16, 30)
            'RGB': (1920, 1080, rs.format.rgb8, 6),
            'depth': None
        },
        '/both/top/': {
            'serial': 'f1371463',
            'hz': 5,
            'RGB': (1920, 1080, rs.format.rgb8, 6),
            'depth': None
        },
    }   

    hz = 10
    if camera is not None:
        hz = cameras[camera]['hz']
    rate = rospy.Rate(hz)
    RGB_topic = f"/cameras/rgb{camera}"
    pub_rgb = rospy.Publisher(RGB_topic, Image, queue_size=10) 
    depth_topic = f"/cameras/depth{camera}"
    pub_depth = rospy.Publisher(depth_topic, Image, queue_size=10) 
    bridge = CvBridge()

    mode = "RGB_DEPTH_"

    pipeline = rs.pipeline()
    config = rs.config()
    if camera is not None:
        if camera not in cameras:
            raise Exception(f"Camera {camera} not found")
        config.enable_device(cameras[camera]['serial'])
        if cameras[camera]['RGB'] is not None:
            print(f"RGB ({RGB_topic}): {cameras[camera]['RGB']}")
            config.enable_stream(rs.stream.color, *cameras[camera]['RGB'])
            mode = "RGB_"
        if cameras[camera]['depth'] is not None:
            print(f"DEPTH ({depth_topic}): {cameras[camera]['depth']}")
            config.enable_stream(rs.stream.depth, *cameras[camera]['depth'])
            mode += "DEPTH_"
    pipeline.start(config)
    working = False

    num_frames = 1
    align = rs.align(rs.stream.color)
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        if mode == "RGB_DEPTH_":
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not depth_frame or not color_frame:
                print(depth_frame, color_frame)
                continue
        elif mode == "RGB_":
            color_frame = frames.get_color_frame()
            depth_frame = None
        elif mode == "DEPTH_":
            color_frame = None
            depth_frame = frames.get_depth_frame()

        if depth_frame is not None:
            depth_image = np.asanyarray(depth_frame.get_data())
            msg = bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
            pub_depth.publish(msg)

        if color_frame is not None:
            color_image = np.asanyarray(color_frame.get_data())
            msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            pub_rgb.publish(msg)

        if depth_frame is not None or color_frame is not None and not working:
            working = True
            print(f"\tPublishing {mode} frames: {camera}")

        rate.sleep()
        if num_frames % (hz*20) == 0:
            print(f"\t{camera} frames: \t{num_frames}")
        num_frames += 1

except Exception as e:
    print(e)
    pass
