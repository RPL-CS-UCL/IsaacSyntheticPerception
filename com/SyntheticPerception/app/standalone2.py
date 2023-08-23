from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({'headless': False})
from core.isaac_handler import IsaacHandler


def main():
    environment = IsaacHandler(1/60, 1/60, simulation_app)
    environment.setup()
    environment.run()

if __name__ == "__main__":
    main()
