#!/usr/bin/env python3

import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation

CAMERA_NAMES = ["ext1", "ext2", "wrist"]

# Maps indices in X_position and X_velocity to their meaning.
POS_DIM_NAMES = ["x", "y", "z", "rot_x", "rot_y", "rot_z"]


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
    )

    blueprint = Blueprint(
        Horizontal(
            Vertical(
                Spatial3DView(name="robot view", origin="/", contents=["/**"]),
                blueprint_row_images(
                    [
                        "cameras/ext1/left",
                        "cameras/ext1/right",
                        "cameras/ext1/depth",
                        "cameras/wrist/left",
                    ]
                ),
                blueprint_row_images(
                    [
                        "cameras/ext2/left",
                        "cameras/ext2/right",
                        "cameras/ext2/depth",
                        "cameras/wrist/right",
                    ]
                ),
                row_shares=[3, 1, 1],
            ),
            Tabs(
                Vertical(
                    *(
                        TimeSeriesView(origin=f'action/cartesian_velocity/{dim_name}') for dim_name in POS_DIM_NAMES
                    ),
                    name='cartesian_velocity',
                ),
                Vertical(
                    TimeSeriesView(origin='action/', contents=['action/gripper_position', 'action/target_gripper_position']),
                    TimeSeriesView(origin='action/gripper_velocity'),
                    name='action/gripper' 
                ),
                Vertical(
                    *(
                        TimeSeriesView(origin=f'action/joint_velocity/{i}') for i in range(6)
                    ),
                    name='action/joint_velocity'
                ),
                Vertical(
                    *(
                        TimeSeriesView(origin=f'robot_state/joint_torques_computed/{i}') for i in range(7)
                    ),
                    name='joint_torques_computed',
                ),
                Vertical(
                    *(
                        TimeSeriesView(origin=f'robot_state/joint_velocities/{i}') for i in range(7)
                    ),
                    name='robot_state/joint_velocities'
                ),
                Vertical(
                    *(
                        TimeSeriesView(origin=f'robot_state/motor_torques_measured/{i}') for i in range(7)
                    ),
                    name='motor_torques_measured',
                ),
            ),
            column_shares=[3, 2],
        ),
        BlueprintPanel(expanded=False),
        SelectionPanel(expanded=False),
        TimePanel(expanded=False),
        auto_space_views=False,
    )
    return blueprint


def link_to_world_transform(
    entity_to_transform: dict[str, tuple[np.ndarray, np.ndarray]],
    joint_angles: list[float],
    link: int,
) -> np.ndarray:

    tot_transform = np.eye(4)
    for i in range(1, link+1):
        entity_path = path_to_link(i)

        start_translation, start_rotation_mat = entity_to_transform[entity_path]

        if i-1 >= len(joint_angles):
            angle_rad = 0
        else:
            angle_rad = joint_angles[i-1]
        vec = np.array(np.array([0, 0, 1]) * angle_rad)
        
        rot = Rotation.from_rotvec(vec).as_matrix()
        rotation_mat = start_rotation_mat @ rot
        
        transform = np.eye(4)
        transform[:3,:3] = rotation_mat
        transform[:3,3] = start_translation
        tot_transform = tot_transform @ transform

    return tot_transform

def log_cartesian_velocity(root: str, cartesian_velocity: np.ndarray):
    if not root.endswith("/"):
        root += "/"

    for vel, name in zip(cartesian_velocity, POS_DIM_NAMES):
        rr.log(root + name, rr.Scalar(vel))


def blueprint_row_images(origins):
    from rerun.blueprint import Horizontal, Spatial2DView

    return Horizontal(
        *(
            Spatial2DView(
                name=org,
                origin=org,
            )
            for org in origins
        ),
    )


def extract_extrinsics(pose: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Takes a vector with dimension 6 and extracts the translation vector and the rotation matrix"""
    translation = pose[:3]
    rotation = Rotation.from_euler("xyz", np.array(pose[3:])).as_matrix()
    return (translation, rotation)


def path_to_link(link: int) -> str:
    return "/".join(f"panda_link{i}" for i in range(link + 1))


def log_angle_rot(
    entity_to_transform: dict[str, tuple[np.ndarray, np.ndarray]],
    link: int,
    angle_rad: float,
) -> None:
    """Logs an angle for the franka panda robot"""
    entity_path = path_to_link(link)

    start_translation, start_rotation_mat = entity_to_transform[entity_path]

    # All angles describe rotations around the transformed z-axis.
    vec = np.array(np.array([0, 0, 1]) * angle_rad)

    rot = Rotation.from_rotvec(vec).as_matrix()
    rotation_mat = start_rotation_mat @ rot

    rr.log(
        entity_path, rr.Transform3D(translation=start_translation, mat3x3=rotation_mat)
    )


# def h5_tree(val, pre=""):
#     # Taken from https://stackoverflow.com/questions/61133916/is-there-in-python-a-single-function-that-shows-the-full-structure-of-a-hdf5-fi
#     items = len(val)
#     for key, val in val.items():
#         items -= 1
#         if items == 0:
#             # the last item
#             if type(val) == h5py._hl.group.Group:
#                 print(pre + "└── " + key)
#                 h5_tree(val, pre + "    ")
#             else:
#                 print(pre + "└── " + key + " (%d)" % len(val))
#         else:
#             if type(val) == h5py._hl.group.Group:
#                 print(pre + "├── " + key)
#                 h5_tree(val, pre + "│   ")
#             else:
#                 print(pre + "├── " + key + " (%d)" % len(val))
