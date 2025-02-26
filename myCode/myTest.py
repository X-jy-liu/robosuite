import numpy as np
import time

from robosuite import make
from robosuite.controllers import controller_factory, load_part_controller_config

env = make(
    env_name="Lift",
    robots="Panda",
    # controller_configs=controller,
    has_renderer=True
)


obs = env.reset()
print(obs.keys())  # See available keys

cube_pos = obs.get("cube_pos")  # Adjust key based on your env
print(f"Cube Position: {cube_pos}")

