# this is a script to automatically run the pipeline to accomplish Objective 1

import json
from fastapi.testclient import TestClient
from app import app  # Your FastAPI LLM app
from config_controller import controller
from myCode.skill_executor import SkillExecutor
from myCode.my_env.multi_object_lift import MultiObjectLift
from fastapi.testclient import TestClient
import numpy as np

from fastapi.testclient import TestClient

with TestClient(app) as client:  # ✅ Triggers startup event
    # --- STEP 1: Load prompt and send command ---
    prompt_path = '/home/jingyang/robosuite/myCode/objective1_prompt_with_better_lift_logics.json'
    with open(prompt_path, "r") as f:
        user_prompt_data = json.load(f)

    # user_command = user_prompt_data["task"]
    user_command = "Put the red cube and the blue cube together"

    response = client.post("/chat_step", params={
        "command": user_command,
        "mode": "override"
    })

    response_json = response.json()
    if "error" in response_json:
        raise RuntimeError(f"LLM API failed: {response_json['error']}")

    symbolic_plan = response_json.get("symbolic_plan", [])
    if not symbolic_plan:
        raise ValueError(f"No symbolic plan returned. Explanation: {response_json.get('explanation', 'N/A')}")

    print("Received command:", user_command)
    print("Generated symbolic plan:")
    print("With the explanation:", response_json.get("explanation", "N/A"))
    print(symbolic_plan)

    # --- STEP 2: Create environment ---
    env = MultiObjectLift(
        robots="Panda",
        controller_configs=controller,
        has_renderer=True
    )
    env.reset()

    for _ in range(10):
        env.step(np.zeros(env.action_dim))
        env.render()

    for obj in env.object_metadata:
        body_id = env.sim.model.body_name2id(obj["name"] + '_main')
        pos = env.sim.data.body_xpos[body_id]
        quat = env.sim.data.body_xquat[body_id]
        print(f"{obj['name']} position: {pos}, quaternion: {quat}")

    # --- STEP 3: Execute plan ---
    executor = SkillExecutor(env)
    executor.execute_plan(symbolic_plan)

    # --- STEP 4: Report positions ---
    obj_info = executor.get_all_object_descriptions()
    print("Final object positions:")
    for obj_name, obj in response_json["current_objects"].items():
        print(f"{obj_name} - Position: {obj['position']}, Shape: {obj['shape']}, Color: {obj['color']}")
