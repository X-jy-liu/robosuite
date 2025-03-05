import robosuite as suite
from robosuite import load_part_controller_config
from robosuite.controllers.composite import REGISTERED_COMPOSITE_CONTROLLERS_DICT

import matplotlib.pyplot as plt
import numpy as np

print(REGISTERED_COMPOSITE_CONTROLLERS_DICT)

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

# Create the environment with a specific camera
env = suite.make(
    "Lift",  # Choose your task
    robots="Panda",  # Choose your robot
    controller_configs=osc_config,
    use_camera_obs=True,  # Enable camera observations
    camera_names="birdview",  # Use the top-down camera
    camera_heights=224,  # Image resolution
    camera_widths=224,
    camera_depths=False,  # No depth, just RGB
)

# Reset environment
obs = env.reset()
print(obs.keys())
print(type(obs))

def is_grayscale(image):
    return np.std(image, axis=2).max() == 0



# Get the top-down view image
topdown_img = obs["birdview_image"]
print(type(topdown_img))
print("Grayscale" if is_grayscale(topdown_img) else "RGB")
# print(np.unique(topdown_img.reshape(-1,3),axis=0))
topdown_img = topdown_img / 255.0
plt.imshow(topdown_img)
plt.axis("off")
plt.show()
# print(topdown_img[10,10,:])

