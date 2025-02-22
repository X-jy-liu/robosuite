import numpy as np
import robosuite as suite

from robosuite.controllers import ALL_PART_CONTROLLERS, PART_CONTROLLER_INFO
from robosuite.controllers import controller_factory

'''
Available Part Controllers: dict_keys(['JOINT_VELOCITY', 'JOINT_TORQUE', 'JOINT_POSITION', 'OSC_POSITION', 'OSC_POSE', 'IK_POSE'])
PART_CONTROLLER_INFO <class 'dict'>
  JOINT_VELOCITY: Joint Velocity
  JOINT_TORQUE: Joint Torque
  JOINT_POSITION: Joint Position
  OSC_POSITION: Operational Space Control (Position Only)
  OSC_POSE: Operational Space Control (Position + Orientation)
  IK_POSE: Inverse Kinematics Control (Position + Orientation) (Note: must have PyBullet installed)
'''

# Print all available part controllers
# print("Available Part Controllers:", ALL_PART_CONTROLLERS)
# print(type(PART_CONTROLLER_INFO))
# # Print detailed information about each controller
# for controller_type, controller_info in PART_CONTROLLER_INFO.items():
#     # print(f"\nController: {controller_type}")
#     print(f"  {controller_type}: {controller_info}")

# # from robosuite.controllers.osc_pose_controller import OSC_POSEController  # Explicitly import OSC_POSE controller

# Define controller
controller_config = suite.load_part_controller_config(default_controller="OSC_POSE")
controller = controller_factory(name="OSC_POSE", config=controller_config)
# controller_config = {
#     "type": "OSC_POSE",
#     "input_max": 1,
#     "input_min": -1,
#     "output_max": [0.05, 0.05, 0.05, 0.5, 0.5, 0.5],
#     "output_min": [-0.05, -0.05, -0.05, -0.5, -0.5, -0.5],
#     "kp": 150,
#     "damping_ratio": 1,
#     "impedance_mode": "fixed",
#     "control_ori": True,
#     "interpolator_pos": None,
#     "interpolator_ori": None,
# }

# Create the environment with the correct controller
env = suite.make(
    env_name="Lift",
    robots="Panda",
    controller_configs=controller_config,
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
)

# Reset the environment
env.reset()

# Command end effector
dpos = np.array([0.01, 0.0, -0.01])
drot = np.array([0.0, 0.0, 0.1])

for i in range(1000):
    action = np.concatenate([dpos, drot])
    obs, reward, done, info = env.step(action)
    env.render()
