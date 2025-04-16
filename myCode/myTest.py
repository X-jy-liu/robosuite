from vanilla_text2action import parse_instruction
from robosuite import make
from robosuite.controllers import load_composite_controller_config


# Define controller
controller = {
    'type': 'BASIC',
    'body_parts': {
        'right': {
            'type': 'OSC_POSE',
            'input_max': 1,
            'input_min': -1,
            'output_max': [0.05, 0.05, 0.05, 0.5, 0.5, 0.5],
            'output_min': [-0.05, -0.05, -0.05, -0.5, -0.5, -0.5],
            'kp': 150, 'damping_ratio': 1,
            'impedance_mode': 'fixed',
            'kp_limits': [0, 300],
            'damping_ratio_limits': [0, 10],
            'position_limits': None,
            'orientation_limits': None,
            'uncouple_pos_ori': True,
            'input_type': 'delta',
            'input_ref_frame': 'base',
            'interpolation': None,
            'ramp_ratio': 0.2,
            'gripper': {'type': 'GRIP'}
        }
    }
}

controller = load_composite_controller_config(controller="BASIC")

# env = make(
#     env_name="Lift",
#     robots="Panda",
#     controller_configs=controller,
#     has_renderer=True,
# )
# env.reset()
# cube_id = env.sim.model.body_name2id("cube_main")


# obs = env.reset()
# # print the cube position
# cube_pos = obs.get("cube_pos", None)  # Extract cube position
# print(f'Cube Pos: {cube_pos}')

# site_id = env.sim.model.site_name2id("cube_default_site")
# site_pos = env.sim.data.site_xpos[site_id]
# print("Cube site position:", site_pos)
# site_ori = env.sim.data.site_xmat[site_id].reshape(3, 3)
# print("Cube site orientation:", site_ori)

from robosuite.environments.manipulation.lift import Lift
from robosuite.models.objects import BoxObject
from robosuite.utils.mjcf_utils import add_to_dict
import numpy as np

class DualCubeLift(Lift):
    def _load_model(self):
        # Load the base model (includes one cube)
        super()._load_model()

        # Create a second cube
        second_cube = BoxObject(
            name="cube2",
            size=[0.02, 0.02, 0.02],
            rgba=[0, 1, 0, 1],  # green cube
        )
        # set the position of the second cube
        second_cube.get_obj().set("pos", "0.35 0.35 0.8")

        # Merge the new object into the simulation model
        self.model.merge_objects([second_cube])

        # Add its body to the worldbody
        add_to_dict(self.model.worldbody, "body", second_cube.get_obj())

    def _get_observations(self, force_update=False):
        obs = super()._get_observations(force_update=force_update)

        # Add second cube’s position
        cube2_pos = self.sim.data.body_xpos[self.sim.model.body_name2id("cube2_main")]
        obs["cube2_pos"] = cube2_pos
        return obs

env = DualCubeLift(
    robots="Panda",
    has_renderer=True,
    controller_configs=controller
)

obs = env.reset()
cube2_pos = obs.get("cube2_pos", None)  # Extract second cube position
print(f'Second Cube Pos: {cube2_pos}')
cube2_pos = env.sim.data.body_xpos[env.sim.model.body_name2id("cube2_main")]
print(f'Second Cube Pos: {cube2_pos}')

for _ in range(100):  # Run for 1000 steps
    action = np.zeros(env.action_dim)  # Constant action to keep the robot still
    obs, reward, done, info = env.step(action)
    env.render()  # This shows the visual output

