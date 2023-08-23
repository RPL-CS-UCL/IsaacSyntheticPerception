# from .sensors import Lidar
from gym.envs.registration import register


register(
    id='isaac-v0',
    entry_point='core.environment:Environment',
    max_episode_steps=300,
)

