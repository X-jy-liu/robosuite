# this is a script to automatically run the pipeline to accomplish Objective 1

import json
from fastapi.testclient import TestClient
from app import app  # Your FastAPI LLM app
from config_controller import controller
from robosuite.environments.manipulation.lift import Lift
from robosuite.models.objects import BoxObject
from robosuite.models.objects.primitive.cylinder import CylinderObject
from robosuite.utils.mjcf_utils import add_to_dict
from myCode.skill_executor import SkillExecutor
import numpy as np

# --- STEP 1: Query LLM API ---
client = TestClient(app)

prompt_path = '/home/jingyang/robosuite/myCode/objective1_prompt_with_better_lift_logics.json'
with open(prompt_path, "r") as f:
    user_prompt = json.load(f)

response = client.post("/generate_plan", json=user_prompt)
symbolic_plan = response.json()  # Expecting list of [action, args]

print("Generated symbolic plan:")
print(symbolic_plan)

# --- STEP 2: Set up environment ---

COLORS = {
    "red": [1, 0, 0, 1],
    "green": [0, 1, 0, 1],
    "blue": [0, 0, 1, 1]
}

class MultiObjectLift(Lift):
    def _load_model(self):
        super()._load_model()
        table_z = self.table_offset[2]
        half_height = 0.0125  # Half height for objects
        cylinder_callibration = 0.0 # Calibration offset for cylinder height
        # checking the height of the table
        print(f"Table height: {table_z}")
        self.object_metadata = []  # Store object info for later access

        # Predefined objects: (name, shape, color_name, position)
        predefined_objects = [
            ("obj0", "cube", "red", [-0.2, -0.2, table_z + half_height]),
            ("obj1", "cube", "blue", [0.2, -0.2, table_z + half_height]),
            ("obj2", "cube", "green", [-0.2, 0.2, table_z + half_height]),
            ("obj3", "cylinder", "red", [0.15, 0.15, table_z + 2*half_height + cylinder_callibration]),
            ("obj4", "cylinder", "blue", [0.05, -0.05, table_z + 2*half_height + cylinder_callibration]),
        ]

        for name, shape, color_name, pos in predefined_objects:
            color_rgba = COLORS[color_name]

            if shape == "cube":
                obj = BoxObject(
                    name=name,
                    size=[0.025, 0.025, 0.025],
                    rgba=color_rgba,
                    material=None,
                    obj_type="all"
                )
            else:
                obj = CylinderObject(
                    name=name,
                    size=[0.025, 0.025],  # (radius, height)
                    rgba=color_rgba,
                    material=None,
                    obj_type="all"
                )

            obj.get_obj().set("pos", f"{pos[0]} {pos[1]} {pos[2]}")

            # Add to the model
            self.model.merge_objects([obj])
            add_to_dict(self.model.worldbody, "body", obj.get_obj())

            # save the object metadata
            self.object_metadata.append({
                "name": name,
                "shape": shape,
                "color": color_name,
                "position": pos
            })

    def _setup_camera(self):
        cam_id = self.sim.model.camera_name2id("birdview")
        self.sim.model.cam_pos[cam_id] = [0.0, 0.0, 1.5]  # Adjust height as needed
        self.sim.model.cam_quat[cam_id] = [1.0, 0.0, 0.0, 0.0]  # Look straight down
        self.sim.model.cam_fovy[cam_id] = 60  # Smaller FOV to zoom in tighter


env = MultiObjectLift(
    robots="Panda",
    controller_configs=controller,
    has_renderer=True)
obs = env.reset()

for _ in range(10):
    env.step(np.zeros(env.action_dim))
    env.render()

# move the inital redish cube out of the table / camera view
for body_name in env.sim.model.body_names:
    if "cube_main" in body_name:
        qpos_addr = env.sim.model.get_joint_qpos_addr("cube_joint0")
        env.sim.data.qpos[qpos_addr[0]:qpos_addr[0]+7] = np.array([5.0, 5.0, 0.0, 1, 0, 0, 0])  # pos + unit quaternion
        env.sim.forward()

# print the objects information
for obj in env.object_metadata:
    body_id = env.sim.model.body_name2id(obj["name"]+'_main') # Append '_main' for MuJoCo body name
    pos = env.sim.data.body_xpos[body_id]
    quat = env.sim.data.body_xquat[body_id]
    print(f"{obj['name']} position: {pos}, quaternion: {quat}")

# --- STEP 3: Execute symbolic plan ---
executor = SkillExecutor(env)
executor.execute_plan(symbolic_plan)
# executor.idle()

obj_pos = executor.get_all_object_positions()
print("Final object positions:")
for obj_name, pos in obj_pos.items():
    print(f"{obj_name}: {pos}")
