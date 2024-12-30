from environment.simEnv import SimEnv
from environment.Memory import Memory
from environment.tasks import Task
from real_world.utils import (
    pix_to_3d_position, get_workspace_crop,
    get_cloth_mask, compute_coverage,
    bound_grasp_pos)
from environment.utils import (
    visualize_action,
    compute_pose,
    pixels_to_3d_positions
)
from real_world.setup import (
    get_top_cam, get_front_cam,
    CLOTHS_DATASET, CURRENT_CLOTH,
    DEFAULT_ORN, DIST_UR5, WS_PC,
    MIN_GRASP_WIDTH, MAX_GRASP_WIDTH,)
from learning.nets import prepare_image
from environment.utils import (
    preprocess_obs,
    add_text_to_image)
from filelock import FileLock
from copy import deepcopy
import numpy as np
from time import time
import os
import h5py
import cv2
from PIL import Image
from typing import List
from itertools import product
import torch

class GraspFailException(Exception):
    def __init__(self):
        super().__init__('Grasp failed due to real world')

class RealWorldEnvInfer():
    def __init__(self,
                 obs_dim: int,
                 num_rotations: int,
                 scale_factors: List[float],
                 pix_grasp_dist: int,
                 reach_distance_limit: float,
                 use_adaptive_scaling: bool,
                 conservative_grasp_radius: int = 4,
                 replace_background=True,
                 **kwargs):
        self.replace_background = replace_background
        self.obs_dim = obs_dim  # what to resize to fit in net
        self.pix_grasp_dist = pix_grasp_dist
        self.conservative_grasp_radius= conservative_grasp_radius
        self.use_adaptive_scaling = use_adaptive_scaling
        # scales
        self.scale_factors = np.array(scale_factors)
        self.adaptive_scale_factors = self.scale_factors.copy()
        # rotations
        self.rotations = [(2*i/(num_rotations-1) - 1) * 90
                          for i in range(num_rotations)]

        # physical limit of dual arm system
        self.left_arm_base = np.array([0.765, 0, 0])
        self.right_arm_base = np.array([-0.765, 0, 0])
        self.reach_distance_limit = reach_distance_limit

        self.action_handlers = {
            'fling': self.pick_and_fling_primitive,
            'drag': self.pick_and_drag_primitive,
            'place': self.pick_and_place_primitive
        }
    
    def pick_and_fling_primitive():
        raise NotImplementedError
    
    def pick_and_drag_primitive():
        raise NotImplementedError
    
    def pick_and_place_primitive():
        raise NotImplementedError

    def get_obs(self):
        cloth_type = 'White_cloth' # 'Pink_shirt' or 'White_cloth'
        cloth_num = 10  # between 1 to 15
        image_path = f"real_world/inference/{cloth_type}/{cloth_num}.png" 
        image_path_full = os.path.join(os.getcwd(), image_path)
        image = Image.open(image_path_full).convert('RGB')
        image.show()  
        if isinstance(image, Image.Image):
            image = np.array(image)

        self.raw_pretransform_rgb, \
            self.raw_pretransform_depth = image, image[...,1]
        print("Observation raw_pretransform_rgb: ", self.raw_pretransform_rgb.shape)
        
        self.postcrop_pretransform_rgb = get_workspace_crop(
            self.raw_pretransform_rgb.copy())
        self.postcrop_pretransform_d = get_workspace_crop(
            self.raw_pretransform_depth.copy())
    
        self.pretransform_rgb = cv2.resize(
            self.postcrop_pretransform_rgb,
            (256, 256))
        self.pretransform_depth = cv2.resize(
            self.postcrop_pretransform_d,
            (256, 256))
        
        cloth_mask = get_cloth_mask(self.pretransform_rgb)
        if self.replace_background:
            cloth_mask = (1-cloth_mask).astype(bool)
            self.pretransform_rgb[..., 0][cloth_mask] = 0
            self.pretransform_rgb[..., 1][cloth_mask] = 0
            self.pretransform_rgb[..., 2][cloth_mask] = 0

        x, y = np.where(cloth_mask == 1)
        dimx, dimy = self.pretransform_depth.shape
        minx = x.min()
        maxx = x.max()
        miny = y.min()
        maxy = y.max()

        pil_image = Image.fromarray(self.pretransform_rgb)
        pil_image.show()        

        return preprocess_obs(
            self.pretransform_rgb.copy(),
            self.pretransform_depth.copy())
    
    def reset(self):
        self.episode_memory = Memory()
        obs = self.get_obs()
        self.transformed_obs = prepare_image(
            obs, self.get_transformations(), self.obs_dim)
        return self.transformed_obs
    
    def infer_single_image(self, policy, img_obs, args):
        # Set the policy to evaluation mode
        policy.eval()
        
        # Perform inference (predict action)
        with torch.no_grad():
            pred_value_maps = policy.act([img_obs])[0]  # predicted value_maps for fling
            action_primitive, action = self.get_max_value_valid_action(pred_value_maps)
        return action_primitive, action 
    
    def get_max_value_valid_action(self, value_maps) -> dict:
        stacked_value_maps = torch.stack(tuple(value_maps.values()))
        print("Stacked_value_maps Shape: ",stacked_value_maps.shape)

        # (**) filter out points too close to edge
        stacked_value_maps = stacked_value_maps[
            :, :,
            self.pix_grasp_dist:-self.pix_grasp_dist,
            self.pix_grasp_dist:-self.pix_grasp_dist]

        # TODO make more efficient by creating index list,
        # flattened value list, then sort and eliminate
        sorted_values, _ = stacked_value_maps.flatten().sort(descending=True)
        actions = list(value_maps.keys())

        for value in sorted_values:
            for indices in np.array(np.where(stacked_value_maps == value)).T:
                # Account for index of filtered pixels. See (**) above
                indices[-2:] += self.pix_grasp_dist

                max_indices = indices[1:]
                x, y, z = max_indices
                action = actions[indices[0]]
                value_map = value_maps[action]
                reach_points = np.array(self.get_action_params(
                    action_primitive=action,
                    max_indices=(x, y, z)))
                # if any point is outside domain, skip
                if any(((p < 0).any() or (p >= self.obs_dim).any())
                       for p in reach_points):
                    continue
                p1, p2 = reach_points[:2]
                print("Actions:", p1, p2)
                action_mask = torch.zeros(value_map.size()[1:])
                action_mask[y, z] = 1
                num_scales = len(self.adaptive_scale_factors)
                rotation_idx = x // num_scales
                scale_idx = x - rotation_idx * num_scales
                scale = self.adaptive_scale_factors[scale_idx]
                rotation = self.rotations[rotation_idx]
                action_kwargs = {
                    'observation': self.transformed_obs[x, ...],
                    'action_primitive': action,
                    'p1': p1,
                    'p2': p2,
                    'scale': scale,
                    'rotation': rotation,
                    'max_indices': max_indices,
                    'action_mask': action_mask,
                    'value_map': value_map[x, :, :],
                    'all_value_maps': value_map,
                    'info': None
                }

                action_kwargs.update({
                    'transformed_depth':
                    action_kwargs['observation'][3, :, :].numpy(),
                    'transformed_rgb':
                    action_kwargs['observation'][:3, :, :].numpy(),
                })
                action_params = self.check_action(
                    pixels=np.array([p1, p2]),
                    **action_kwargs)
                if not action_params['valid_action']:
                    continue
                reachable, left_or_right = self.check_action_reachability(
                    action=action,
                    p1=action_params['p1'],
                    p2=action_params['p2'])

                if not reachable:
                    continue
                action_kwargs['action_visualization'] =\
                    action_params['get_action_visualization_fn']()
                
                # self.log_step_stats(action_kwargs)
                pil_image  = Image.fromarray(action_kwargs['action_visualization'])
                pil_image.show()

                print("action_kwargs['pretransform_pixels']: ", action_params['pretransform_pixels'])

                for k in ['valid_action',
                          'pretransform_pixels',
                          'get_action_visualization_fn']:
                    del action_params[k]
                return action_kwargs['action_primitive'], action_params
        return None, None

    def log_step_stats(self, action):
        self.episode_memory.add_observation(action['observation'])
        self.episode_memory.add_action(action['action_mask'])
        self.episode_memory.add_value(
            key='action_visualization',
            value=action['action_visualization'])
        self.episode_memory.add_value(
            key='rotation', value=float(action['rotation']))
        self.episode_memory.add_value(
            key='scale', value=float(action['scale']))
        self.episode_memory.add_value(
            key='value_map',
            value=action['value_map'])
        self.episode_memory.add_value(
            key='action_primitive',
            value=action['action_primitive'])
        self.episode_memory.add_value(
            key='max_indices',
            value=np.array(action['max_indices']))
        for key, value in self.current_task.get_stats().items():
            self.episode_memory.add_value(
                key=key,
                value=value)
        if self.dump_visualizations:
            if action['all_value_maps'] is not None:
                self.episode_memory.add_value(
                    key='value_maps',
                    value=action['all_value_maps'])
            self.episode_memory.add_value(
                key='all_obs',
                value=self.transformed_obs)
    
    def compute_coverage(self):
        coverage = compute_coverage(rgb=self.get_obs()[0])
        print(
            f"\tCoverage: {coverage/CLOTHS_DATASET[CURRENT_CLOTH]['flatten_area']:.04f}")
        return coverage
    
    def get_transformations(self):
        return list(product(
            self.rotations, self.adaptive_scale_factors))
    
    def get_action_params(self, action_primitive, max_indices):
        x, y, z = max_indices
        if action_primitive == 'fling':
            center = np.array([x, y, z])
            p1 = center[1:].copy()
            p1[0] = p1[0] + self.pix_grasp_dist
            p2 = center[1:].copy()
            p2[0] = p2[0] - self.pix_grasp_dist
        else:
            raise Exception(
                f'Action Primitive not supported: {action_primitive}')
        return p1, p2

    def check_arm_reachability(self, arm_base, reach_pos):
        #return np.linalg.norm(arm_base - reach_pos) < self.reach_distance_limit
        # TODO
        return True
    
    def check_action_reachability(
            self, action: str, p1: np.array, p2: np.array):
        if action == 'fling' or action == 'stretchdrag':
            # right and left must reach each point respectively
            return self.check_arm_reachability(self.left_arm_base, p1) \
                and self.check_arm_reachability(self.right_arm_base, p2), None
        elif action == 'drag' or action == 'place':
            # either right can reach both or left can reach both
            if self.check_arm_reachability(self.left_arm_base, p1) and\
                    self.check_arm_reachability(self.left_arm_base, p2):
                return True, 'left'
            elif self.check_arm_reachability(self.right_arm_base, p1) and \
                    self.check_arm_reachability(self.right_arm_base, p2):
                return True, 'right'
            else:
                return False, None
        raise NotImplementedError()
    

    def check_action(self, action_primitive, pixels,
                     transformed_depth, transformed_rgb,
                     scale, rotation,
                     value_map=None, all_value_maps=None,
                     **kwargs):
        args = {
            'pretransform_depth': self.pretransform_depth.copy(),
            'pretransform_rgb': self.pretransform_rgb.copy(),
            'transformed_depth': transformed_depth.copy(),
            'transformed_rgb': transformed_rgb.copy(),
            'scale': scale,
            'rotation': rotation
        }
        retval = pixels_to_3d_positions(
            pixels=pixels,
            pose_matrix=compute_pose(
                pos=[0, 2, 0],
                lookat=[0, 0, 0],
                up=[0, 0, 1]), **args)

        def get_action_visualization():
            return visualize_action(
                action_primitive=action_primitive,
                transformed_pixels=pixels,
                pretransform_pixels=retval['pretransform_pixels'],
                value_map=value_map,
                all_value_maps=all_value_maps,
                **args)

        retval.update({
            'get_action_visualization_fn': get_action_visualization
        })

        cloth_mask = (self.pretransform_depth != 2.0).astype(np.uint8)
        pix_1, pix_2 = retval['pretransform_pixels']
        if self.conservative_grasp_radius > 0:
            grasp_mask_1 = np.zeros(cloth_mask.shape)
            grasp_mask_1 = cv2.circle(
                img=grasp_mask_1,
                center=(pix_1[1], pix_1[0]),
                radius=self.conservative_grasp_radius,
                color=1, thickness=-1).astype(bool)
            grasp_mask_2 = np.zeros(cloth_mask.shape)
            grasp_mask_2 = cv2.circle(
                img=grasp_mask_2,
                center=(pix_2[1], pix_2[0]),
                radius=self.conservative_grasp_radius,
                color=1, thickness=-1).astype(bool)
            retval.update({
                'p1_grasp_cloth': cloth_mask[grasp_mask_1].all(),
                'p2_grasp_cloth': cloth_mask[grasp_mask_2].all(),
            })
        else:
            retval.update({
                'p1_grasp_cloth': True,
                'p2_grasp_cloth': True,
            })
        return retval
