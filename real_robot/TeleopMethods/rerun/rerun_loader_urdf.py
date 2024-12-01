#!/usr/bin/env python3
"""
Modified version of the URDF logger
"""
from __future__ import annotations

import argparse
import os
import pathlib
from typing import Optional

from PIL import Image
import numpy as np
import rerun as rr  # pip install rerun-sdk
import scipy.spatial.transform as st
import trimesh
from urdf_parser_py import urdf as urdf_parser

class URDFLogger:
    """Class to log a URDF to Rerun."""

    def __init__(self, filepath: str, root_path: str = "") -> None:
        self.urdf = urdf_parser.URDF.from_xml_file(filepath)
        self.mat_name_to_mat = {mat.name: mat for mat in self.urdf.materials}
        self.entity_to_transform = {}
        self.root_path = root_path

    def link_entity_path(self, link: urdf_parser.Link) -> str:
        """Return the entity path for the URDF link."""
        root_name = self.urdf.get_root()
        link_names = self.urdf.get_chain(root_name, link.name)[0::2]  # skip the joints
        return "/".join(link_names)

    def joint_entity_path(self, joint: urdf_parser.Joint) -> str:
        """Return the entity path for the URDF joint."""
        root_name = self.urdf.get_root()
        link_names = self.urdf.get_chain(root_name, joint.child)[0::2]  # skip the joints
        return "/".join(link_names)

    def log(self) -> None:
        """Log a URDF file to Rerun."""
        rr.log(self.root_path + "", rr.ViewCoordinates.RIGHT_HAND_Z_UP, timeless=True)  # default ROS convention

        for joint in self.urdf.joints:
            entity_path = self.joint_entity_path(joint)
            self.log_joint(entity_path, joint)

        for link in self.urdf.links:
            entity_path = self.link_entity_path(link)
            self.log_link(entity_path, link)

    def log_link(self, entity_path: str, link: urdf_parser.Link) -> None:
        # create one mesh out of all visuals
        for i, visual in enumerate(link.visuals):
            self.log_visual(entity_path + f"/visual_{i}", visual)

    def log_joint(self, entity_path: str, joint: urdf_parser.Joint) -> None:
        translation = rotation = None

        if joint.origin is not None and joint.origin.xyz is not None:
            translation = joint.origin.xyz

        if joint.origin is not None and joint.origin.rpy is not None:
            rotation = st.Rotation.from_euler("xyz", joint.origin.rpy).as_matrix()

        self.entity_to_transform[self.root_path + entity_path] = (translation, rotation)
        # print(f"Path: {self.root_path + entity_path}: transform: {translation}, {rotation}")
        # rr.log(self.root_path + entity_path, rr.Transform3D(translation=translation, mat3x3=rotation), timeless=True)
        rr.log(self.root_path + entity_path, rr.Transform3D(translation=translation, mat3x3=rotation))

    def log_visual(self, entity_path: str, visual: urdf_parser.Visual) -> None:
        material = None
        if visual.material is not None:
            if visual.material.color is None and visual.material.texture is None:
                # use globally defined material
                material = self.mat_name_to_mat[visual.material.name]
            else:
                material = visual.material

        transform = np.eye(4)
        if visual.origin is not None and visual.origin.xyz is not None:
            transform[:3, 3] = visual.origin.xyz
        if visual.origin is not None and visual.origin.rpy is not None:
            transform[:3, :3] = st.Rotation.from_euler("xyz", visual.origin.rpy).as_matrix()

        if isinstance(visual.geometry, urdf_parser.Mesh):
            resolved_path = resolve_ros_path(visual.geometry.filename)
            mesh_scale = visual.geometry.scale
            mesh_or_scene = trimesh.load_mesh(resolved_path)
            if mesh_scale is not None:
                transform[:3, :3] *= mesh_scale
        elif isinstance(visual.geometry, urdf_parser.Box):
            mesh_or_scene = trimesh.creation.box(extents=visual.geometry.size)
        elif isinstance(visual.geometry, urdf_parser.Cylinder):
            mesh_or_scene = trimesh.creation.cylinder(
                radius=visual.geometry.radius,
                height=visual.geometry.length,
            )
        elif isinstance(visual.geometry, urdf_parser.Sphere):
            mesh_or_scene = trimesh.creation.icosphere(
                radius=visual.geometry.radius,
            )
        else:
            rr.log(self.root_path + 
                "",
                rr.TextLog("Unsupported geometry type: " + str(type(visual.geometry))),
            )
            mesh_or_scene = trimesh.Trimesh()
        
        # print(f"Path: {entity_path}: transform: {transform}")
        mesh_or_scene.apply_transform(transform)

        if isinstance(mesh_or_scene, trimesh.Scene):
            scene = mesh_or_scene
            # use dump to apply scene graph transforms and get a list of transformed meshes
            for i, mesh in enumerate(scene.dump()):
                if material is not None:
                    if material.color is not None:
                        mesh.visual = trimesh.visual.ColorVisuals()
                        mesh.visual.vertex_colors = material.color.rgba
                    elif material.texture is not None:
                        texture_path = resolve_ros_path(material.texture.filename)
                        mesh.visual = trimesh.visual.texture.TextureVisuals(image=Image.open(texture_path))
                log_trimesh(self.root_path + entity_path+f"/{i}", mesh)
        else:
            mesh = mesh_or_scene
            if material is not None:
                if material.color is not None:
                    mesh.visual = trimesh.visual.ColorVisuals()
                    mesh.visual.vertex_colors = material.color.rgba
                elif material.texture is not None:
                    texture_path = resolve_ros_path(material.texture.filename)
                    mesh.visual = trimesh.visual.texture.TextureVisuals(image=Image.open(texture_path))
            log_trimesh(self.root_path + entity_path, mesh)


def log_trimesh(entity_path: str, mesh: trimesh.Trimesh) -> None:
    vertex_colors = albedo_texture = vertex_texcoords = None
    if isinstance(mesh.visual, trimesh.visual.color.ColorVisuals):
        vertex_colors = mesh.visual.vertex_colors
    elif isinstance(mesh.visual, trimesh.visual.texture.TextureVisuals):
        albedo_texture = mesh.visual.material.baseColorTexture
        if len(np.asarray(albedo_texture).shape) == 2:
            # If the texture is grayscale, we need to convert it to RGB.
            albedo_texture = np.stack([albedo_texture] * 3, axis=-1)
        vertex_texcoords = mesh.visual.uv
        # Trimesh uses the OpenGL convention for UV coordinates, so we need to flip the V coordinate
        # since Rerun uses the Vulkan/Metal/DX12/WebGPU convention.
        if vertex_texcoords is None:
            pass
        else:
            vertex_texcoords[:, 1] = 1.0 - vertex_texcoords[:, 1]
    else:
        # Neither simple color nor texture, so we'll try to retrieve vertex colors via trimesh.
        try:
            colors = mesh.visual.to_color().vertex_colors
            if len(colors) == 4:
                # If trimesh gives us a single vertex color for the entire mesh, we can interpret that
                # as an albedo factor for the whole primitive.
                mesh_material = Material(albedo_factor=np.array(colors))
            else:
                vertex_colors = colors
        except Exception:
            pass
    
    rr.log(
        entity_path,
        rr.Mesh3D(
            vertex_positions=mesh.vertices,
            triangle_indices=mesh.faces,
            vertex_normals=mesh.vertex_normals,
            vertex_colors=vertex_colors,
            albedo_texture=albedo_texture,
            vertex_texcoords=vertex_texcoords,
        ),
        timeless=True,
    )

def resolve_ros_path(path: str) -> str:
    """Resolve a ROS path to an absolute path."""
    if path.startswith("package://"):
        path = pathlib.Path(path)
        package_name = path.parts[1]
        relative_path = pathlib.Path(*path.parts[2:])

        if package_name == "ur_description":
            return os.path.join("./URDF/UR", relative_path)
        Exception("Package not found")
        return None
