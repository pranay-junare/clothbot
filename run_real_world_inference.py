# python run_real_world_inference.py  --eval --load flingbot.pth
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
from real_world.realWorldEnvInfer import RealWorldEnvInfer
from typing import List
from itertools import product


if __name__ == '__main__':
    args = config_parser().parse_args()
    env = RealWorldEnvInfer(**vars(args))
    policy, _, _ = setup_network(args)

    obs = env.reset()
    print("Observation shape: ", obs.shape)
    action = env.infer_single_image(policy, obs, args)
    
    print("Predicted action for the input image:", action)