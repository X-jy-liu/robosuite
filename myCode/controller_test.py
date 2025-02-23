import robosuite as suite
from robosuite.models.robots import Panda
from robosuite.models.grippers import PandaGripper
import mujoco  # New official mujoco bindings

# Load the Panda robot model
robot_model = Panda()

# Initialize the gripper for the Panda robot
grippers = [PandaGripper()]

# Load the MuJoCo XML model of the robot
mjcf_model = robot_model.get_model()

# Convert the MJCF model to a simulation model
model = mujoco.MjModel.from_xml_string(mjcf_model.from_xml_string())
data = mujoco.MjData(model)  # Create simulation data structure

from robosuite.controllers import composite_controller_factory

# Try initializing a BASIC composite controller
controller = composite_controller_factory("BASIC", model, robot_model, grippers)

# Print to verify that the controller is successfully created
print("Initialized controller:", controller)
