import torch
import os
from utils import config_parser, setup_network, setup_envs
from tensorboardX import SummaryWriter
from copy import copy
import ray
from time import time
from PIL import Image
import numpy as np
from torchvision import transforms

class Inference():
    def __init__(self,
                pix_grasp_dist:int,
                **kwargs):
        # primitives parameters
        self.pix_grasp_dist = pix_grasp_dist

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

    def get_max_value_valid_action(self, value_maps) -> dict:
        print("self.pix_grasp_dist: ",self.pix_grasp_dist)
        print(value_maps)
        # print(torch.stack(list(value_maps.values())).shape)  # Fix here

        stacked_value_maps = torch.stack(tuple(value_maps.values()))
        print(stacked_value_maps.shape)

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
                if action == 'place' or action == 'drag':
                    action_kwargs['left_or_right'] = left_or_right

                if action == 'stretchdrag':
                    left_start_drag_pos = action_params['p1']
                    right_start_drag_pos = action_params['p2']
                    left_start_drag_pos[1] = self.grasp_height
                    right_start_drag_pos[1] = self.grasp_height

                    # compute drag direction
                    drag_direction = np.cross(
                        left_start_drag_pos - right_start_drag_pos,
                        np.array([0, 1, 0]))
                    drag_direction = self.stretchdrag_dist * \
                        drag_direction / np.linalg.norm(drag_direction)

                    left_end_drag_pos = left_start_drag_pos + drag_direction
                    right_end_drag_pos = right_start_drag_pos + drag_direction

                    final_drag_reachable =\
                        self.check_arm_reachability(
                            self.left_arm_base, left_end_drag_pos)\
                        and self.check_arm_reachability(
                            self.right_arm_base, right_end_drag_pos)
                    reachable = final_drag_reachable and reachable

                if not reachable:
                    continue
                action_kwargs['action_visualization'] =\
                    action_params['get_action_visualization_fn']()
                self.log_step_stats(action_kwargs)
                for k in ['valid_action',
                          'pretransform_pixels',
                          'get_action_visualization_fn']:
                    del action_params[k]
                return action_kwargs['action_primitive'], action_params
        return None, None

    def infer_single_image(self, policy, img_obs, args):
        # Set the policy to evaluation mode
        policy.eval()

        # Transform the image to the format expected by the model
        transform = transforms.Compose([
            transforms.Resize((224, 224)),  # Resize to the input size expected by the model
            transforms.ToTensor(),          # Convert to tensor
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])  # Normalize if required
        ])
        img_obs = transform(img_obs).unsqueeze(0)  # Add batch dimension (1, C, H, W)

        # Move the img_obs to the device (GPU or CPU)
        device = next(policy.parameters()).device  # Get the device of the policy
        img_obs = img_obs.to(device)
        print("Image shape: ", img_obs.shape)
        print("Image size: ", len(img_obs.size()))
        
        # Perform inference (predict action)
        with torch.no_grad():
            pred_value_maps = policy.act([img_obs])  # predicted value_maps for fling
            action_primitive, action = self.get_max_value_valid_action(pred_value_maps)
        return None 


if __name__ == '__main__':
    args = config_parser().parse_args()
    inference = Inference (**vars(args))
    policy, _, _ = setup_network(args)

    # Load your single input image (make sure the image path is correct)
    image_path = "/home/hong_data/flingbot/real_world/inference/cloth1.png"
    image = Image.open(image_path).convert('RGB')  # Ensure image is in RGB format
    action = inference.infer_single_image(policy, image, args)
    
    print("Predicted action for the input image:", action)