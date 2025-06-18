# this is a script to automatically run the pipeline to accomplish Objective 1

import json
from fastapi.testclient import TestClient
from app import app  # Your FastAPI LLM app
from config_controller import controller
from myCode.skill_executor import SkillExecutor
from myCode.multi_object_lift import MultiObjectLift
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

# --- STEP 2: Create the environment and reset it ---
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
