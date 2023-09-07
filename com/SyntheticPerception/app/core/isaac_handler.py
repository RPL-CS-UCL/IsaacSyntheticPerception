import argparse
import pathlib
import sys
import functools
import datetime
import socket
import os
import shutil

from omni.isaac.core import World
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
import time

to_np = lambda x: x.detach().cpu().numpy()


class IsaacHandler:
    def __init__(self, physics_dt, render_dt, simulation_app) -> None:
        self.simulation_app = simulation_app

        enable_extension("omni.kit.asset_converter")
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
        self.start_time = None

    def setup(self):
        self._appwindow = omni.appwindow.get_default_app_window()

        # self.env.setup()

    def step(self, render):
        if self._needs_reset:
            self._needs_reset = False
            self.env.reset()

        # make the env step the environment
        self.env.step()
        # self.env._world.step(render=render)

    def create_session_name(self, config):
        now = datetime.datetime.now()
        # session_name = 'session_{}_{:04d}_{:02d}_{:02d}_{:02d}_{:02d}_{:02d}_{}'.format(
        #     socket.gethostname(), now.year, now.month, now.day, now.hour, now.minute, now.second, config.tag)
        # session_name = os.path.join((config.output_path), session_name)
        session_name = 'session_{:04d}_{:02d}_{:02d}_{:02d}_{:02d}_{:02d}_{}'.format(
        now.year, now.month, now.day, now.hour, now.minute, now.second, config.tag)
        session_name = os.path.join((config.train_path), session_name)
        os.makedirs(session_name)
        return session_name

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
        print(" ======= ", len(train_envs))
        train_envs[0].setup_objects_agents_goals()
        eval_envs = [make("eval") for _ in range(config.envs)]
        train_envs = [Damy(env) for env in train_envs]
        eval_envs = [Damy(env) for env in eval_envs]
        acts = train_envs[0].action_space
        config.num_actions = acts.n if hasattr(acts, "n") else acts.shape[0]

        state = None
        prefill = config.prefill
        print(f"Prefill dataset ({prefill} steps).")
        print(acts)
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
            # self.step(render)
            state = tools.isaac_simulate(
                random_agent,
                train_envs[0],
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
            agent = Dreamer(
                train_envs[0].observation_space,
                train_envs[0].action_space,
                config,
                logger,
                train_dataset,
            ).to(config.device)
            agent.requires_grad_(requires_grad=False)
            if (logdir / "latest_model.pt").exists():
                agent.load_state_dict(torch.load(logdir / "latest_model.pt"))
                agent._should_pretrain._once = False

            # make sure eval will be executed once after config.steps
            while agent._step < config.steps + config.eval_every:
                logger.write()
                if config.eval_episode_num > 0:
                    print("Start evaluation.")
                    eval_policy = functools.partial(agent, training=False)
                    tools.isaac_simulate(
                        eval_policy,
                        eval_envs[0],
                        eval_eps,
                        config.evaldir,
                        logger,
                        is_eval=True,
                        episodes=config.eval_episode_num,
                    )
                    if config.video_pred_log:
                        video_pred = agent._wm.video_pred(next(eval_dataset))
                        logger.video("eval_openl", to_np(video_pred))
                print("Start training.")
                state = tools.isaac_simulate(
                    agent,
                    train_envs[0],
                    train_eps,
                    config.traindir,
                    logger,
                    limit=config.dataset_size,
                    steps=config.eval_every,
                    state=state,
                )
                torch.save(agent.state_dict(), logdir / "latest_model.pt")
            for env in train_envs + eval_envs:
                try:
                    env.close()
                except Exception:
                    pass

            if not self._world.is_simulating():
                self._needs_reset = True

    def test_agent(self, config):
        tools.set_seed_everywhere(config.seed)
        if config.deterministic_run:
            tools.enable_deterministic_run()
        logdir = pathlib.Path().expanduser()
        logdir = self.create_session_name(config)

        logdir = config.agent_path
        logdir = pathlib.Path(logdir)
        shutil.copy(config.environ_path, logdir)
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
        make = lambda mode, id: dreamer_fns.make_env_seq(config, mode, id)
        train_envs = [make("train", id) for id in range(config.envs)]

        world = World(
            stage_units_in_meters=1.0,
            physics_dt=1 / 60,
            rendering_dt=1 / 60,
        )
        for i in range(len(train_envs)):
            train_envs[i].setup_objects_agents_goals(
                world=world,
                id=i,
                cone_path=config.cone_asset,
                sensor_path=config.sensor_asset,
                mat_path =config.mat_path,
                obstacle_path = config.obstacle_path
            )
        train_envs[0].setup_light(skybox_path = config.skybox_path)
        eval_envs = [make("eval", _ + len(train_envs)) for _ in range(1)]

        for i in range(len(eval_envs)):
            eval_envs[i].setup_objects_agents_goals(
                world=world,
                id=i + len(train_envs) + 1,
                cone_path=config.cone_asset,
                sensor_path=config.sensor_asset,

                mat_path =config.mat_path
            )
        train_envs = [Damy(env) for env in train_envs]
        eval_envs = [Damy(env) for env in eval_envs]
        acts = train_envs[0].action_space
        config.num_actions = acts.n if hasattr(acts, "n") else acts.shape[0]

        state = None
        prefill = config.prefill
        # sort out this while loop
        while self.simulation_app.is_running():
            render = True
            # self.step(render)

            print("Simulate agent.")
            agent = Dreamer(
                train_envs[0].observation_space,
                train_envs[0].action_space,
                config,
                logger,
                None,
            ).to(config.device)
            agent.requires_grad_(requires_grad=False)
            if (logdir / "latest_model.pt").exists():
                print(" _+_+_+_+_+_+_+_+_+_+_+_ LOADING")
                print(" loagdir", logdir)
                agent.load_state_dict(torch.load(logdir / "latest_model.pt"))
                agent._should_pretrain._once = False

            # make sure eval will be executed once after config.steps
            while agent._step < config.steps + config.eval_every:
                logger.write()
                self.start_time = time.time()

                eval_policy = functools.partial(agent, training=False)
                state = tools.run_agent(
                    eval_policy,
                    train_envs,
                    train_eps,
                    config.traindir,
                    logger,
                    start_time=self.start_time,
                    limit=config.dataset_size,
                    steps=config.eval_every,
                    state=state,
                )
            if not self._world.is_simulating():
                self._needs_reset = True

    def test_agent2(self, config):
        tools.set_seed_everywhere(config.seed)
        if config.deterministic_run:
            tools.enable_deterministic_run()

        logdir = config.agent_path
        logdir = pathlib.Path(logdir)
        shutil.copy(config.environ_path, logdir)
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
        make = lambda mode, id: dreamer_fns.make_env_seq(config, mode, id)
        train_envs = [make("train", id) for id in range(config.envs)]

        world = World(
            stage_units_in_meters=1.0,
            physics_dt=1 / 60,
            rendering_dt=1 / 60,
        )
        for i in range(len(train_envs)):
            train_envs[i].setup_objects_agents_goals(
                world=world,
                id=i,
                cone_path=config.cone_asset,
                sensor_path=config.sensor_asset,
                mat_path =config.mat_path,
                obstacle_path = config.obstacle_path
            )
        train_envs[0].setup_light(skybox_path = config.skybox_path)
        eval_envs = [make("eval", _ + len(train_envs)) for _ in range(1)]

        for i in range(len(eval_envs)):
            eval_envs[i].setup_objects_agents_goals(
                world=world,
                id=i + len(train_envs) + 1,
                cone_path=config.cone_asset,
                sensor_path=config.sensor_asset,

                mat_path =config.mat_path
            )
        train_envs = [Damy(env) for env in train_envs]
        eval_envs = [Damy(env) for env in eval_envs]
        acts = train_envs[0].action_space
        config.num_actions = acts.n if hasattr(acts, "n") else acts.shape[0]

        state = None
        prefill = config.prefill
        prefill = 0  
        print(f"Prefill dataset ({prefill} steps).")
        # print(acts)
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
            # self.step(render)
            print(f"Logger: ({logger.step} steps).")

            print("Simulate agent.")
            train_dataset = dreamer_fns.make_dataset(train_eps, config)
            eval_dataset = dreamer_fns.make_dataset(eval_eps, config)
            agent = Dreamer(
                train_envs[0].observation_space,
                train_envs[0].action_space,
                config,
                logger,
                train_dataset,
            ).to(config.device)
            agent.requires_grad_(requires_grad=False)
            if (logdir / "latest_model.pt").exists():
                agent.load_state_dict(torch.load(logdir / "latest_model.pt"))
                agent._should_pretrain._once = False

            # make sure eval will be executed once after config.steps
            while agent._step < config.steps + config.eval_every:
                logger.write()
                self.start_time = time.time()
                print("Start training.")
                state = tools.simulate_multi_test(
                    agent,
                    train_envs,
                    train_eps,
                    config.traindir,
                    logger,
                    start_time=self.start_time,
                    limit=config.dataset_size,
                    steps=config.eval_every,
                    state=state,
                )
                torch.save(agent.state_dict(), logdir / "latest_model.pt")
            for env in train_envs + eval_envs:
                try:
                    env.close()
                except Exception:
                    pass

            if not self._world.is_simulating():
                self._needs_reset = True
    def run3(self, config):
        tools.set_seed_everywhere(config.seed)
        if config.deterministic_run:
            tools.enable_deterministic_run()
        logdir = pathlib.Path().expanduser()
        logdir = self.create_session_name(config)
        logdir = pathlib.Path(logdir)

        # logdir = config.agent_path
        # logdir = pathlib.Path(logdir)
        shutil.copy(config.environ_path, logdir)
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
        make = lambda mode, id: dreamer_fns.make_env_seq(config, mode, id)
        train_envs = [make("train", id) for id in range(config.envs)]

        world = World(
            stage_units_in_meters=1.0,
            physics_dt=1 / 60,
            rendering_dt=1 / 60,
        )
        for i in range(len(train_envs)):
            train_envs[i].setup_objects_agents_goals(
                world=world,
                id=i,
                cone_path=config.cone_asset,
                sensor_path=config.sensor_asset,
                mat_path =config.mat_path,
                obstacle_path = config.obstacle_path
            )
        train_envs[0].setup_light(skybox_path = config.skybox_path)
        eval_envs = [make("eval", _ + len(train_envs)) for _ in range(1)]

        for i in range(len(eval_envs)):
            eval_envs[i].setup_objects_agents_goals(
                world=world,
                id=i + len(train_envs) + 1,
                cone_path=config.cone_asset,
                sensor_path=config.sensor_asset,

                mat_path =config.mat_path
            )
        train_envs = [Damy(env) for env in train_envs]
        eval_envs = [Damy(env) for env in eval_envs]
        acts = train_envs[0].action_space
        config.num_actions = acts.n if hasattr(acts, "n") else acts.shape[0]

        if config.curriculum_learning:
            print("====CURRICULUM LEARNING====")
            for env in range(len(train_envs)):
                train_envs[env].num_obstacles = 7
                train_envs[env].random_starting_orientation = False
                train_envs[env].size_of_map = 10
                train_envs[env].minimum_distance_between_objects = 15
        state = None
        prefill = config.prefill
        print(f"Prefill dataset ({prefill} steps).")
        # print(acts)
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
            # self.step(render)
            state = tools.simulate_multi(
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
            agent = Dreamer(
                train_envs[0].observation_space,
                train_envs[0].action_space,
                config,
                logger,
                train_dataset,
            ).to(config.device)
            agent.requires_grad_(requires_grad=False)
            if (logdir / "latest_model.pt").exists():
                agent.load_state_dict(torch.load(logdir / "latest_model.pt"))
                agent._should_pretrain._once = False

            # make sure eval will be executed once after config.steps
            while agent._step < config.steps + config.eval_every:
                logger.write()
                if config.eval_episode_num > 0:
                    print("Start evaluation.")
                    eval_policy = functools.partial(agent, training=False)
                    tools.simulate_multi(
                        eval_policy,
                        eval_envs,
                        eval_eps,
                        config.evaldir,
                        logger,
                        is_eval=True,
                        episodes=config.eval_episode_num,
                    )
                    if config.video_pred_log:
                        video_pred = agent._wm.video_pred(next(eval_dataset))
                        logger.video("eval_openl", to_np(video_pred))
                self.start_time = time.time()
                print("Start training.")
                state = tools.simulate_multi(
                    agent,
                    train_envs,
                    train_eps,
                    config.traindir,
                    logger,
                    start_time=self.start_time,
                    limit=config.dataset_size,
                    steps=config.eval_every,
                    state=state,
                )
                print("=======WE ARE HERE========", state)
                torch.save(agent.state_dict(), logdir / "latest_model.pt")
            for env in train_envs + eval_envs:
                try:
                    env.close()
                except Exception:
                    pass

            if not self._world.is_simulating():
                self._needs_reset = True

    def run2(self, config):
        tools.set_seed_everywhere(config.seed)
        if config.deterministic_run:
            tools.enable_deterministic_run()
        logdir = pathlib.Path().expanduser()
        logdir = pathlib.Path("/home/jon/Documents/Isaac_dreamer/isaac_training")

        logdir = "/home/jon/Documents/Isaac_dreamer/train"
        logdir = pathlib.Path(logdir)
        # logdir = "/home/stuart/Documents/isaac_training"
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
        print(" ======= ", len(train_envs))

        print("There will be ", config.steps, " steps")

        world = World(
            stage_units_in_meters=1.0,
            physics_dt=1 / 60,
            rendering_dt=1 / 60,
        )
        train_envs[0].setup_objects_agents_goals(world=world, id=0)
        eval_envs = [make("eval") for _ in range(config.envs)]
        train_envs = [Damy(env) for env in train_envs]
        eval_envs = [Damy(env) for env in eval_envs]
        acts = train_envs[0].action_space
        config.num_actions = acts.n if hasattr(acts, "n") else acts.shape[0]

        state = None
        prefill = config.prefill
        print(f"Prefill dataset ({prefill} steps).")
        print(acts)
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
            start_time = time.time()

            render = True
            # self.step(render)
            state = tools.simulate(
                random_agent,
                train_envs,
                train_eps,
                config.traindir,
                logger,
                limit=config.dataset_size,
                steps=prefill,
            )

            time_taken = time.time() - start_time
            print("*********************")
            print("1k sims took ", time_taken)
            logger.step += prefill * config.action_repeat
            print(f"Logger: ({logger.step} steps).")

            print("Simulate agent.")
            train_dataset = dreamer_fns.make_dataset(train_eps, config)
            eval_dataset = dreamer_fns.make_dataset(eval_eps, config)
            agent = Dreamer(
                train_envs[0].observation_space,
                train_envs[0].action_space,
                config,
                logger,
                train_dataset,
            ).to(config.device)
            agent.requires_grad_(requires_grad=False)
            if (logdir / "latest_model.pt").exists():
                agent.load_state_dict(torch.load(logdir / "latest_model.pt"))
                agent._should_pretrain._once = False

            # make sure eval will be executed once after config.steps
            while agent._step < config.steps + config.eval_every:
                logger.write()
                if config.eval_episode_num > 0:
                    print("Start evaluation.")
                    eval_policy = functools.partial(agent, training=False)
                    tools.simulate(
                        eval_policy,
                        eval_envs,
                        eval_eps,
                        config.evaldir,
                        logger,
                        is_eval=True,
                        episodes=config.eval_episode_num,
                    )
                    if config.video_pred_log:
                        video_pred = agent._wm.video_pred(next(eval_dataset))
                        logger.video("eval_openl", to_np(video_pred))
                print("Start training.")
                state = tools.simulate(
                    agent,
                    train_envs,
                    train_eps,
                    config.traindir,
                    logger,
                    limit=config.dataset_size,
                    steps=config.eval_every,
                    state=state,
                )
                torch.save(agent.state_dict(), logdir / "latest_model.pt")
            for env in train_envs + eval_envs:
                try:
                    env.close()
                except Exception:
                    pass

            if not self._world.is_simulating():
                self._needs_reset = True

    def camera_test(config):
        pass
