import argparse
import pathlib
import sys

from omni.isaac.kit import SimulationApp

from pxr import Usd, Gf, UsdGeom
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    is_stage_loading,
    update_stage_async,
    update_stage,
)
import omni
from omni.isaac.core import World
from omni.isaac.quadruped.robots import Anymal
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.core.utils.nucleus import get_assets_root_path
from pxr import Gf, UsdGeom
import time
import omni.appwindow  # Contains handle to keyboard
import numpy as np

import torch
from torch import nn 
from torch import distributions as torchd
import gym

import carb
from omni.isaac.core.utils.extensions import enable_extension
from core.environment import Environment
from core.objects import Object
from core.rig import Agent 
import dreamer.dreamer_copy as dreamer_fns
from dreamer.dreamer_copy import Dreamer
from dreamer.dreamer_copy import tools
from dreamer.parallel import Damy

class IsaacHandler:
    def __init__(self, physics_dt, render_dt, simulation_app) -> None:
        self.simulation_app = simulation_app


        enable_extension('omni.kit.asset_converter')
        self.simulation_app.update()
        # self._world = World(
        #     stage_units_in_meters=1.0,
        #     physics_dt=physics_dt,
        #     rendering_dt=render_dt,
        # )
        self._stage = omni.usd.get_context().get_stage()
        # self._world.reset()
        self._needs_reset = False

        self.env = gym.make("isaac-v0")
        # Environment("sim")

    def setup(self):

        self._appwindow = omni.appwindow.get_default_app_window()
      
        #self.env.setup()


    def step(self, render):
        if self._needs_reset:
            self._needs_reset = False
            self.env.reset()
        
        # make the env step the environment
        self.env.step()
        # self.env._world.step(render=render)

    def run(self, config):
        print(config)
        tools.set_seed_everywhere(config.seed)
        if config.deterministic_run:
            tools.enable_deterministic_run()
        print(pathlib.Path())
        logdir = pathlib.Path().expanduser()
        config.traindir = config.traindir or logdir / "train_eps"
        config.evaldir = config.evaldir or logdir / "eval_eps"
        config.steps //= config.action_repeat
        config.eval_every //= config.action_repeat
        config.log_every //= config.action_repeat
        config.time_limit //= config.action_repeat

        print("Logdir", logdir)
        logdir.mkdir(parents=True, exist_ok=True)
        config.traindir.mkdir(parents=True, exist_ok=True)
        config.evaldir.mkdir(parents=True, exist_ok=True)
        step = dreamer_fns.count_steps(config.traindir)
        # step in logger is environmental step
        logger = tools.Logger(logdir, config.action_repeat * step)

        print("Create envs.")
        directory = config.traindir
        train_eps = tools.load_episodes(directory, limit=config.dataset_size)
        directory = config.evaldir
        eval_eps = tools.load_episodes(directory, limit=1)
        make = lambda mode: dreamer_fns.make_env(config, mode)
        train_envs = [make("train") for _ in range(config.envs)]
        eval_envs = [make("eval") for _ in range(config.envs)]
        train_envs = [Damy(env) for env in train_envs]
        eval_envs = [Damy(env) for env in eval_envs]
        acts = train_envs[0].action_space
        config.num_actions = acts.n if hasattr(acts, "n") else acts.shape[0]

        state = None
        prefill = config.prefill
        print(f"Prefill dataset ({prefill} steps).")
        if hasattr(acts, "discrete"):
            random_actor = tools.OneHotDist(
                torch.zeros(config.num_actions).repeat(config.envs, 1)
            )
        else:
            random_actor = torchd.independent.Independent(
                torchd.uniform.Uniform(
                    torch.Tensor(acts.low).repeat(config.envs, 1),
                    torch.Tensor(acts.high).repeat(config.envs, 1),
                ),
                1,
            )
        
        def random_agent(o, d, s):
            action = random_actor.sample()
            logprob = random_actor.log_prob(action)
            return {"action": action, "logprob": logprob}, None
        
        # sort out this while loop
        while self.simulation_app.is_running():
            render = True
            #self.step(render)
            state = tools.isaac_simulate(
            random_agent,
            train_envs,
            train_eps,
            config.traindir,
            logger,
            limit=config.dataset_size,
            steps=prefill,
            )
            logger.step += prefill * config.action_repeat
            print(f"Logger: ({logger.step} steps).")

            print("Simulate agent.")
            train_dataset = dreamer_fns.make_dataset(train_eps, config)
            eval_dataset = dreamer_fns.make_dataset(eval_eps, config)


            if not self._world.is_simulating():
                self._needs_reset = True
