import os
import cv2  # Optional, you can also use plt.imsave()
import robosuite as suite
from robosuite import load_composite_controller_config
from robosuite.models.objects.primitive.box import BoxObject
from robosuite.models.objects.primitive.cylinder import CylinderObject
from robosuite.utils.mjcf_utils import add_to_dict
from robosuite.environments.manipulation.lift import Lift
import matplotlib.pyplot as plt
import numpy as np

# osc_config = load_composite_controller_config(controller='BASIC')

osc_config = {
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

# Directory to save images
os.makedirs("randomized_images", exist_ok=True)

# Object pool settings
SHAPES = ["cube", "cylinder"]
COLORS = {
    "red": [1, 0, 0, 1],
    "green": [0, 1, 0, 1],
    "blue": [0, 0, 1, 1]
}

# Custom environment with 5 randomized objects
class MultiObjectLift(Lift):
    def _load_model(self):
        super()._load_model()
        table_z = self.table_offset[2]
        self.robots[0].robot_model.set_base_xpos([-2, 0, 0])
        
        # Create and add 5 randomized objects
        for i in range(5):
            shape = np.random.choice(SHAPES)
            color_name = np.random.choice(list(COLORS.keys()))
            color_rgba = COLORS[color_name]

            # Choose shape
            if shape == "cube":
                obj = BoxObject(
                    name=f"obj{i}",
                    size=[0.025, 0.025, 0.025],
                    rgba=color_rgba,
                    material=None,
                    obj_type="all"
                )
            else:
                obj = CylinderObject(
                    name=f"obj{i}",
                    size=[0.025, 0.025],  # (radius, height)
                    rgba=color_rgba,
                    material=None,
                    obj_type="all"
                )

            # Random position in X-Y plane
            pos_xy = np.random.uniform(low=-0.35, high=0.35, size=2)
            pos = [pos_xy[0], pos_xy[1], table_z + 0.02]  # 0.02 = object half-height

            obj.get_obj().set("pos", f"{pos[0]} {pos[1]} {pos[2]}")

            # Add to the model
            self.model.merge_objects([obj])
            add_to_dict(self.model.worldbody, "body", obj.get_obj())

# Image generation loop
num_samples = 5

for i in range(num_samples):
    env = MultiObjectLift(
        robots="Panda",
        controller_configs=osc_config,
        has_renderer=True,
        camera_names="birdview",
        camera_heights=512,
        camera_widths=512,
        camera_depths=False
    )
    # reset the environment
    env.reset()
    env.sim.forward()
    # TODO: remove the body with body_names == cube_main out of the table by changing its position
    for body_name in env.sim.model.body_names:
        if "cube_main" in body_name:
            qpos_addr = env.sim.model.get_joint_qpos_addr("cube_joint0")
            env.sim.data.qpos[qpos_addr[0]:qpos_addr[0]+7] = np.array([5.0, 5.0, 0.0, 1, 0, 0, 0])  # pos + unit quaternion
            env.sim.forward()

    image = env.sim.render(camera_name="birdview", width=512, height=512)
    plt.imsave(f"randomized_images/scene_{i+1}.png", image)

    print(f"Saved scene {i+1}!")

print("All randomized object scenes saved.")

