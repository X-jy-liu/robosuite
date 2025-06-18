from fastapi import FastAPI
from pydantic import BaseModel
from typing import List, Dict
import openai
import os
import ast  # for safe evaluation of string output from LLM
import threading
import time
from robosuite.environments.manipulation.lift import Lift
from myCode.skill_executor import SkillExecutor
from config_controller import controller
from robosuite.models.objects import BoxObject, CylinderObject
from robosuite.utils.mjcf_utils import add_to_dict
from myCode.multi_object_lift import MultiObjectLift
import numpy as np
# ----------------------------
# Step 1: create the Sim simulator
# ----------------------------
app = FastAPI()

# ----------------------------
# Step 2: Define input format
# ----------------------------
class FunctionSpec(BaseModel):
    params: List[str]
    modes: List[str] = []

class ObjectSpec(BaseModel):
    name: str
    shape: str
    color: str
    position: List[float]
    size: float

class PromptInput(BaseModel):
    task: str
    environment: Dict[str, List[ObjectSpec]]
    available_functions: Dict[str, FunctionSpec]
    examples: List[dict] = []
    instructions: str = ""


# -----------------------------------
# Step 3: Build prompt from JSON input
# -----------------------------------
def construct_prompt(payload: PromptInput) -> str:
    # Format objects
    obj_descriptions = "\n".join([
        f"- {obj.name}: shape={obj.shape}, color={obj.color}, position={obj.position}, size={obj.size}"
        for obj in payload.environment.get("objects", [])
    ])

    # Format functions
    func_descriptions = "\n".join([
        f"- {name}({', '.join(spec.params)}) -> modes: {spec.modes}"
        for name, spec in payload.available_functions.items()
    ])

    # Format examples (if any)
    example_blocks = ""
    if payload.examples:
        formatted_examples = []
        for ex in payload.examples:
            ex_task = ex.get("task", "")
            ex_plan = ex.get("plan", [])
            plan_str = "\n  ".join([str(tuple(step)) for step in ex_plan])
            formatted_examples.append(f'Example Task: "{ex_task}"\n  Plan:\n  {plan_str}')
        example_blocks = "\n\n" + "\n\n".join(formatted_examples)

    # Final prompt string
    prompt = f"""
        Task: {payload.task}

        Environment Objects:
        {obj_descriptions}

        Available Functions:
        {func_descriptions}

        Instructions:
        {payload.instructions.strip()}

        {example_blocks}

        Now, generate a symbolic plan for this task:
        {payload.task}

        Respond ONLY with a valid Python list of tuples.
        """

    return prompt.strip()


# -----------------------------------
# Step 4: Query GPT-4 or similar LLM
# -----------------------------------
def call_llm(prompt: str):
    openai.api_key = os.getenv("OPENAI_API_KEY")

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.5
    )

    content = response["choices"][0]["message"]["content"]
    return ast.literal_eval(content.strip())  # Safely convert string to Python object

class SimWrapper:
    def __init__(self):
        self.env = MultiObjectLift(
            robots="Panda",
            controller_configs=controller,
            has_renderer=False  # Set to True only if needed
        )
        self.env.reset()

        # Move the default cube away
        for body_name in self.env.sim.model.body_names:
            if "cube_main" in body_name:
                qpos_addr = self.env.sim.model.get_joint_qpos_addr("cube_joint0")
                self.env.sim.data.qpos[qpos_addr[0]:qpos_addr[0]+7] = np.array([5.0, 5.0, 0.0, 1, 0, 0, 0])
                self.env.sim.forward()

        self.executor = SkillExecutor(self.env)

    def get_current_state(self):
        return self.executor.get_all_object_positions()

    def execute_plan(self, plan):
        print("Executing:", plan)
        self.executor.execute_plan(plan)

# Global state
sim = SimWrapper()
session = {"running": False}
SESSION = {
    "base_prompt": None,
    "history": []
}
log = []

# ----------------------------
# API Endpoints
# ----------------------------
# ----------------------------
# API Endpoints
# ----------------------------
@app.post("/init_session")
def init_session(payload: PromptInput):
    SESSION["base_prompt"] = payload
    SESSION["history"] = []
    return {"status": "Session initialized", "task": payload.task}

@app.post("/chat_step")
def chat_step(command: str):
    if not SESSION["base_prompt"]:
        return {"error": "Session not initialized. Use /init_session first."}

    payload = SESSION["base_prompt"]
    payload.instructions += f"\nFollow-up: {command}"
    SESSION["history"].append(command)

    raw_state = sim.get_current_state()
    object_specs = [
        ObjectSpec(
            name=name,
            shape="cube" if "cube" in name else "cylinder",
            color="red",
            position=pos.tolist(),
            size=0.025
        ) for name, pos in raw_state.items()
    ]
    payload.environment["objects"] = object_specs

    prompt = construct_prompt(payload)
    try:
        plan = call_llm(prompt)
    except Exception as e:
        return {"error": f"LLM failed: {e}"}

    sim.execute_plan(plan)

    return {
        "command": command,
        "symbolic_plan": plan,
        "current_objects": raw_state
    }

@app.get("/")
def read_root():
    return {"message": "Interactive Planning API in http://127.0.0.1:8000/docs. Use /init_session then /chat_step."}
