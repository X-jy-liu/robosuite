import mujoco
import mujoco.viewer  # Import the Mujoco viewer
import time
import numpy as np


from robosuite.models.arenas import TableArena
from robosuite.models import MujocoWorldBase
from robosuite.models.robots import Panda
from robosuite.models.grippers import gripper_factory
from robosuite.models.objects import BallObject

# Initialize the Mujoco world
world = MujocoWorldBase()
mujoco_robot = Panda()

# Add gripper to robot
gripper = gripper_factory('PandaGripper')
mujoco_robot.add_gripper(gripper)
mujoco_robot.set_base_xpos([0, 0, 0])
world.merge(mujoco_robot)

# Add a table arena
mujoco_arena = TableArena()
mujoco_arena.set_origin([0.8, 0, 0])
world.merge(mujoco_arena)

# Add an object (ball)
sphere = BallObject(
    name="sphere",
    size=[0.04],
    rgba=[0, 0.5, 0.5, 1]).get_obj()
sphere.set('pos', '1.0 0 1.0')
world.worldbody.append(sphere)

# Create Mujoco model and data
model = world.get_model(mode="mujoco")
data = mujoco.MjData(model)

# Open the viewer window
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    while data.time < 5:  # Run for 5 seconds
        mujoco.mj_step(model, data)
        viewer.sync()  # Update the visualization
        time.sleep(0.01)  # Simulate real-time speed