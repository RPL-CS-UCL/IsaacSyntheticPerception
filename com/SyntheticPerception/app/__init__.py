# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import importlib
import sys

print('[CUSTOM] Reloading...')
L = list(sys.modules.keys())
for k in L:
    if 'com.copycat' in k:
        print(k)
        importlib.reload(sys.modules[k])
from .synthetic_perception import SyntheticPerception
from .synthetic_perception_extension import SyntheticPerceptionExtension

# from .sensors import Lidar
from gym.envs.registration import register


register(
    id='isaac-v0',
    entry_point='core.environment:Environment',
    max_episode_steps=300,
)
