import numpy as np
import time
from robosuite import make
from robosuite.controllers import controller_factory, load_part_controller_config

# Load OSC_POSE controller configuration
osc_config = load_part_controller_config(
    custom_fpath="/home/jingyang/robosuite/robosuite/controllers/config/default/parts/osc_pose.json",
    default_controller="OSC_POSE"
)

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

# Initialize environment
env = make(
    env_name="Lift",
    robots="Panda",
    controller_configs=controller,
    has_renderer=True,
)

# Function to move the end-effector (EE) to a target position
def move_ee(target_pos, grip_action=0, threshold=0.01, max_steps=500):

    """ Move end-effector to a target position with optional gripping action. """
    for _ in range(max_steps):
        obs = env._get_observations()  # Get latest observation without resetting
        current_pos = obs["robot0_eef_pos"]
        error = target_pos - current_pos

        # Control command: [dx, dy, dz, droll, dpitch, dyaw, gripper]
        action = np.concatenate([error * 3, [0, 0, 0], [grip_action]])

        # Step the simulation
        obs, _, _, _ = env.step(action)
        env.render()

        # Check if target position is reached
        if np.linalg.norm(error) < threshold:
            print(f'Success! Reached {target_pos}')
            return True
    return False

# Function to control gripper
def grip(state):
    """ Control gripper: -1 (open), 1 (close) """
    action = np.array([0, 0, 0, 0, 0, 0, state])  # Only affect gripper
    for _ in range(10):  # Apply action multiple times for stability
        env.step(action)
        env.render()

# **Get Cube Position**
obs = env.reset()
initial_ee_pos = obs.get("robot0_eef_pos", None)
cube_pos = obs.get("cube_pos", None)  # Extract cube position

if cube_pos is None:
    raise ValueError("Cube position not found in observation!")

print(f"Cube Position: {cube_pos}")

# Adjusted target pick position based on cube position
target_pick_pos = np.array([cube_pos[0], cube_pos[1], cube_pos[2]])  # Slightly above cube

# Move above object
move_ee(target_pick_pos + np.array([0, 0, 0.1]))  

# Open gripper
grip(-1)
time.sleep(1)

# Move down to grasp object
move_ee(target_pick_pos)

# Close gripper (grasp)
grip(1)
time.sleep(1)

# Lift object
move_ee(target_pick_pos + np.array([0, 0, 0.1]))

target_drop_pos = np.array([cube_pos[0]+0.1, cube_pos[1]+0.3, cube_pos[2]])

# Hover above the drop-off location
hover_target_drop_pos = target_drop_pos + np.array([0, 0, 0.1])
move_ee(hover_target_drop_pos)

# Drop the cube
move_ee(target_drop_pos)
grip(-1)
time.sleep(1)

# Initialize the gripper
move_ee(hover_target_drop_pos)
move_ee(initial_ee_pos)

# Close environment
env.close()