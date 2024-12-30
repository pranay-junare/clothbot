import rerun as rr
import numpy as np
import cv2
import time
import rosbag
from cv_bridge import CvBridge
from rerun_loader_urdf import URDFLogger
from scipy.spatial.transform import Rotation

rr.init(f"rosbag({time.time})", spawn=True)

bridge = CvBridge()

init={}

messages = set()

# rr.stdout()
urdf_logger = URDFLogger("./URDF/dual.urdf", "URDF/")
urdf_logger.log()
lighting_joint_names = []
thunder_joint_names = []
for path in urdf_logger.entity_to_transform.keys():
    if "lightning" in path and path.endswith("link") and not path.endswith("base_link"):
        lighting_joint_names.append(path)
    if "thunder" in path and path.endswith("link") and not path.endswith("base_link"):
        thunder_joint_names.append(path)
lighting_joint_names.sort(key=len)
thunder_joint_names.sort(key=len)
# print(urdf_logger.urdf.joints)

for key,value in urdf_logger.entity_to_transform.items():
    if key not in lighting_joint_names and key not in thunder_joint_names:
        rr.log(key, rr.Transform3D(translation=value[0], mat3x3=value[1]), static=True)
urdf_init = False

def blueprint_raw():
    from rerun.blueprint import (
        Blueprint,
        BlueprintPanel,
        Horizontal,
        Vertical,
        SelectionPanel,
        Spatial3DView,
        TimePanel,
        TimeSeriesView,
        Tabs,
        Spatial2DView,
    )
    expanded = False

    blueprint = Blueprint(
        Vertical(
            Horizontal(
                Tabs(
                    Spatial3DView(origin="trajectory/3d/thunder", 
                                  name="Arm Trajectory (Thunder)",
                                  background=(100, 150, 200)),
                    Spatial2DView(origin="camera/rgb/thunder/wrist",)
                                  
                ),
                Tabs(
                    Spatial2DView(origin="camera/top_depth"),
                    Spatial2DView(origin="camera/top_rgb"),
                    TimeSeriesView(origin="graphs/both"),
                ),

                Tabs(
                    Spatial3DView(origin="trajectory/3d/lightning", 
                                  name="Arm Trajectory (Lightning)",
                                  background=(100, 150, 200)),
                    Spatial3DView(origin="URDF", 
                                  name="URDF Viewer",
                                  background=(100, 150, 200)),
                    Spatial2DView(origin="camera/rgb/lightning/wrist",)
                ),
            ),
            Horizontal(
                Tabs(
                    TimeSeriesView(origin="thunder/joints"),
                    TimeSeriesView(origin="thunder/cartesian"),
                    TimeSeriesView(origin="thunder/velocity"),
                    TimeSeriesView(origin="thunder/force_raw"),
                    TimeSeriesView(origin="thunder/torque"),
                    TimeSeriesView(origin="thunder/force"),
                ),
                Tabs(
                    TimeSeriesView(origin="lightning/joints"),
                    TimeSeriesView(origin="lightning/cartesian"),
                    TimeSeriesView(origin="lightning/velocity"),
                    TimeSeriesView(origin="lightning/force_raw"),
                    TimeSeriesView(origin="lightning/torque"),
                    TimeSeriesView(origin="lightning/force"),
                ),
            ),
        ),
        BlueprintPanel(expanded=expanded),
        SelectionPanel(expanded=expanded),
        TimePanel(expanded=expanded),
        auto_space_views=False,
    )
    return blueprint

rr.send_blueprint(blueprint_raw())

# <origin xyz="0.3715 0 0" rpy="2.208 1.561 0.622"/> # lightning
# <origin xyz="-0.3715 0 0" rpy="1.5708 -1.571 0.000"/> # thunder
lightning_rot = [ # 3x3 rotation matrix
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
    [0.0, 1.0, 0.0]
]
thunder_rot = [ # 3x3 rotation matrix
    [0.8778253, -0.2032892,  0.4337008],
    [0.2032892, -0.6617427, -0.7216440],
    [0.4337008,  0.7216440, -0.5395680],
]
rr.log("trajectory/3d/lightning", rr.Transform3D(
    translation=[0.3715, 0, 0], 
    mat3x3=lightning_rot))
rr.log("trajectory/3d/thunder", rr.Transform3D(
    translation=[-0.3715, 0, 0], 
    mat3x3=thunder_rot))

cartesian_points = {}
cartesian_color = {}

for topic, msg, t in rosbag.Bag('test.bag').read_messages():
    messages.add(topic)

    rr.set_time_seconds("real_time", seconds=t.to_sec())

    if topic == "/cameras/rgb/lightning/wrist":
        rr.log("camera/rgb/lightning/wrist", rr.Image(bridge.imgmsg_to_cv2(msg)))
    if topic == "/cameras/rgb/thunder/wrist":
        rr.log("camera/rgb/thunder/wrist", rr.Image(bridge.imgmsg_to_cv2(msg)))

    if topic == "/cameras/top/rgb":
        rr.log("camera/top_rgb", rr.Image(bridge.imgmsg_to_cv2(msg)))
    # if topic == "/cameras/top/depth":
    #     raw = np.frombuffer(msg.data, dtype=np.uint16)
    #     image = raw.reshape(msg.height, msg.width)
    #     rr.log("camera/top_depth", rr.Image(image))
        # rr.log("camera/top_depth", rr.Image(bridge.imgmsg_to_cv2(msg)))

    cartesian_topics = {
        "/lightning_cartesian_eef": "trajectory/3d/lightning/data",
        "/thunder_cartesian_eef": "trajectory/3d/thunder/data"
    }
    if topic in cartesian_topics:
        name = cartesian_topics[topic]
        if name not in cartesian_points:
            cartesian_points[name] = ([], [], [])
        cartesian_points[name][0].append((msg.data[:3]))
        cartesian_points[name][1].append(cartesian_color.get(name, (255, 255, 255)))
        cartesian_points[name][2].append(0.01*cartesian_color.get(name, (0,0,0))[0]/255.0*10)
        rr.log(name+"/now", rr.Points3D([msg.data[:3]], radii=0.02))


    cartesian_topics = {
        "/lightning_cartesian_eef": "lightning/cartesian",
        "/thunder_cartesian_eef": "thunder/cartesian"
    }
    if topic in cartesian_topics:
        name = cartesian_topics[topic]
        if topic not in init: 
            init[topic] = True
            names = ["x", "y", "z"]
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
            for i in range(3):
                rr.log(f"{name}/{names[i]}", rr.SeriesLine(name=f"{names[i]}", color=colors[i], width=2), static=True)
        for i in range(3):
            rr.log(f"{name}/{names[i]}", rr.Scalar(msg.data[i]))


    spark_angle_topics = {
        "/lightning_q": "lightning/joints",
        "/thunder_q": "thunder/joints"
    }
    if topic in spark_angle_topics:
        paths = lighting_joint_names if topic == "/lightning_q" else thunder_joint_names
        for i, angle_rad in enumerate(msg.data):
            start_translation, start_rotation_mat = urdf_logger.entity_to_transform[paths[i]]
            # if paths[i].endswith("upper_arm_link"):
            #     vec = np.array(np.array([1, 0, 0]) * angle_rad)
            # else:
            vec = np.array(np.array([0, 0, 1]) * angle_rad)
            rot = Rotation.from_rotvec(vec).as_matrix()
            rr.log(paths[i], rr.Transform3D(mat3x3=rot, translation=start_translation))


        name = spark_angle_topics[topic]
        if topic not in init: 
            init[topic] = True
            for i in range(len(msg.data)):
                rr.log(f"{name}/joint{i}", rr.SeriesLine(name=f"joint{i}", width=2), static=True)
        for i in range(len(msg.data)):
            rr.log(f"{name}/joint{i}", rr.Scalar(msg.data[i]))

    force_topics = {
        "/lightning_ft": "lightning/force",
        "/thunder_ft": "thunder/force"
    }
    if topic in force_topics:
        name = force_topics[topic]
        data = list(msg.data)[:3]
        data.append(np.sum(np.abs(data)))
        if topic not in init: 
            init[topic] = True
            forces = ["x", "y", "z", "total"]
            # colors = ["red", "green", "blue", "white"]
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255)]
            for i in range(len(data)):
                rr.log(f"{name}/force{forces[i]}", rr.SeriesLine(
                    name=f"{forces[i]}", color=colors[i], width=2), static=True)
        for i in range(len(data)):
            rr.log(f"{name}/force{forces[i]}", rr.Scalar(data[i]))
        
        if name == "lightning/force":
            force = data[-1]/60.0*255
            cartesian_color["trajectory/3d/lightning/data"] = (force, 0, 0)
        if name == "thunder/force":
            force = data[-1]/60.0*255
            cartesian_color["trajectory/3d/thunder/data"] = (force, 0, 0)

    torque_topics = {
        "/lightning_ft": "lightning/torque",
        "/thunder_ft": "thunder/torque"
    }
    if topic in torque_topics:
        name = torque_topics[topic]
        torques = list(msg.data)[3:6]
        data.append(np.sum(np.abs(data)))
        torque = ["roll", "pitch", "yaw"]
        if name not in init: 
            init[name] = True
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255)]
            for i in range(len(torques)):
                rr.log(f"{name}/torque{torque[i]}", rr.SeriesLine(
                    name=f"{torque[i]}", color=colors[i], width=2), static=True)
        for i in range(len(torques)):
            rr.log(f"{name}/torque{torque[i]}", rr.Scalar(torques[i]))


    force_topics = {
        "/lightning_raw_ft_raw": "lightning/force_raw",
        "/thunder_raw_ft_raw": "thunder/force_raw"
    }
    if topic in force_topics:
        name = force_topics[topic]
        data = list(msg.data)[:3]
        data.append(np.sum(np.abs(data)))
        if topic not in init: 
            init[topic] = True
            forces = ["x", "y", "z", "total"]
            # colors = ["red", "green", "blue", "white"]
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255)]
            for i in range(len(data)):
                rr.log(f"{name}/force{forces[i]}", rr.SeriesLine(
                    name=f"{forces[i]}", color=colors[i], width=2), static=True)
        for i in range(len(data)):
            rr.log(f"{name}/force{forces[i]}", rr.Scalar(data[i]))




    velocity_topics = {
        "/lightning_speed": "lightning/velocity",
        "/thunder_speed": "thunder/velocity"
    }
    if topic in velocity_topics:
        name = velocity_topics[topic]
        data = list(msg.data)[:3]
        data.append(np.sum(np.abs(data)))
        if topic not in init: 
            init[topic] = True
            forces = ["x", "y", "z", "total"]
            # colors = ["red", "green", "blue", "white"]
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255)]
            for i in range(len(data)):
                rr.log(f"{name}/velocity{forces[i]}", rr.SeriesLine(
                    name=f"{forces[i]}", color=colors[i], width=2), static=True)
        for i in range(len(data)):
            rr.log(f"{name}/velocity{forces[i]}", rr.Scalar(data[i]))

        if 'graphs/both' not in init:
            init['graphs/both'] = True
            for arm in ['lightning', 'thunder']:
                rr.log(f"graphs/both/{arm}_velocity", rr.SeriesLine(name=f"{arm}_velocity", width=2), static=True)
            # rr.log("graphs/both/velocitytotal", rr.SeriesLine(name="velocitytotal", width=2), static=True)
        if name == "lightning/velocity":
            data = np.array(data)
            data = np.linalg.norm(data)
            rr.log(f"graphs/both/lightning_velocity", rr.Scalar(data))
        if name == "thunder/velocity":
            data = np.array(data)
            data = np.linalg.norm(data)
            rr.log(f"graphs/both/thunder_velocity", rr.Scalar(data)) 

            
for name, data in cartesian_points.items():
    points, colors, radii = data[0], data[1], 0.01
    # print(len(colors))
    # print(len(points))
    if len(points) > 0:
        points3d = points[::5]
        # rr.Radius.ui_points(5.0)
        # rr.log(name, rr.Points3D(points3d, radii=radii, colors=colors), static=True)
        rr.log(name, rr.LineStrips3D([points3d], colors=colors), static=True)
# print(messages)