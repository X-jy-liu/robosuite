import os
import cv2  # Optional, you can also use plt.imsave()
import robosuite as suite
from robosuite import load_part_controller_config
from robosuite.controllers.composite import REGISTERED_COMPOSITE_CONTROLLERS_DICT

import matplotlib.pyplot as plt
import numpy as np

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

def randomize_cube(env):
    # Random color
    random_color = np.concatenate([np.random.rand(3), [1.0]])
    for name in ["cube_g0", "cube_g0_vis"]:
        geom_id = env.sim.model.geom_name2id(name)
        env.sim.model.geom_rgba[geom_id] = random_color

    # Random size (shape)
    random_size = np.random.uniform(low=0.02, high=0.05, size=3)
    for name in ["cube_g0", "cube_g0_vis"]:
        geom_id = env.sim.model.geom_name2id(name)
        env.sim.model.geom_size[geom_id] =   random_size

    # Random position for the cube_main body
    cube_body_id = env.sim.model.body_name2id("cube_main")
    random_pos = np.array([
        np.random.uniform(-0.5, 0.5),   # x range (adjust based on your workspace)
        np.random.uniform(-0.5, 0.5),   # y range
        np.random.uniform(0.8, 1.0)     # z range (above the table)
    ])
    env.sim.model.body_pos[cube_body_id] = random_pos

    # Random orientation as a normalized quaternion
    random_quat = np.random.randn(4)  # Sample random quaternion
    random_quat /= np.linalg.norm(random_quat)  # Normalize it to unit quaternion
    env.sim.model.body_quat[cube_body_id] = random_quat

    env.sim.forward()  # Apply the changes to the simulation

    # # Optional: Random mass
    # cube_body_id = env.sim.model.body_name2id("cube")
    # env.sim.model.body_mass[cube_body_id] = np.random.uniform(0.1, 1.0)

    env.sim.forward()

env = suite.make(
    "Lift",  # Choose your task
    robots="Panda",  # Choose your robot
    controller_configs=osc_config,
    use_camera_obs=True,  # Enable camera observations
    camera_names="birdview",  # Use the top-down camera
    camera_heights=512,  # Image resolution
    camera_widths=512,
    camera_depths=False,  # No depth, just RGB
)

num_samples = 5  # Number of randomizations/images you want to generate

for i in range(num_samples):
    obs = env.reset()
    randomize_cube(env)  # Apply randomization

    # Directly get the updated observation with the camera image
    obs = env._get_observations()  # No need for env.sim.render()

    # Get the top-down camera image (uint8 RGB)
    topdown_img = obs["birdview_image"]

    # Optional: convert to float [0,1] or save directly
    img_float = topdown_img / 255.0

    # Save using matplotlib (keeps RGB)
    plt.imsave(f"randomized_images/random_cube_{i}.png", img_float)

    print(f"Saved randomized image {i}!")

print("All images saved.")
