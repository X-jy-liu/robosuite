import robosuite
import numpy as np
from robosuite.controllers import load_composite_controller_config

# BASIC controller: arms controlled using OSC, mobile base (if present) using JOINT_VELOCITY, other parts controlled using JOINT_POSITION 
controller_config = load_composite_controller_config(controller="BASIC")
print(controller_config)

# create an environment to visualize on-screen
env = robosuite.make(
    "TwoArmLift",
    robots=["Sawyer", "Panda"],             # load a Sawyer robot and a Panda robot
    gripper_types="default",                # use default grippers per robot arm
    controller_configs=controller_config,   # arms controlled via OSC, other parts via JOINT_POSITION/JOINT_VELOCITY
    env_configuration="opposed",            # (two-arm envs only) arms face each other
    has_renderer=True,                      # on-screen rendering
    render_camera="frontview",              # visualize the "frontview" camera
    has_offscreen_renderer=False,           # no off-screen rendering
    control_freq=20,                        # 20 hz control for applied actions
    horizon=200,                            # each episode terminates after 200 steps
    use_object_obs=False,                   # no observations needed
    use_camera_obs=False,                   # no observations needed
)

# reset the environment
env.reset()

for i in range(10):
    action = np.random.randn(*env.action_spec[0].shape) * 0.1
    obs, reward, done, info = env.step(action)  # take action in the environment
    env.render()  # render on display

env.close
print(obs)