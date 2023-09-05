import argparse
import pathlib
import sys

import ruamel.yaml as yaml
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({'headless': False})
from core.isaac_handler import IsaacHandler
import dreamer.tools as tools


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--configs", nargs="+")
    args, remaining = parser.parse_known_args()
    print(pathlib.Path(sys.argv[0]).parent / "configs.yaml")
    configs = yaml.safe_load(
        (pathlib.Path(sys.argv[0]).parent / "configs.yaml").read_text()
    )
    
    user_configs= yaml.safe_load(
        (pathlib.Path(sys.argv[0]).parent / "user_config.yaml").read_text()
    )
    for key in user_configs: 
        configs["defaults"][key] =user_configs[key]#None#configs2.train_path
    def recursive_update(base, update):
        for key, value in update.items():
            if isinstance(value, dict) and key in base:
                recursive_update(base[key], value)
            else:
                base[key] = value

    name_list = ["defaults", *args.configs] if args.configs else ["defaults"]
    defaults = {}
    for name in name_list:
        recursive_update(defaults, configs[name])
    parser = argparse.ArgumentParser()
    for key, value in sorted(defaults.items(), key=lambda x: x[0]):
        arg_type = tools.args_type(value)
        parser.add_argument(f"--{key}", type=arg_type, default=arg_type(value))

    environment = IsaacHandler(1/60, 1/60, simulation_app)
    environment.setup()
    environment.run3(parser.parse_args(remaining))
    # environment.test_agent2(parser.parse_args(remaining))

if __name__ == "__main__":
    main()
