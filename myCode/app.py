from fastapi import FastAPI
from pydantic import BaseModel
from typing import List, Dict
import openai
import os
import ast
import json
from pathlib import Path
import numpy as np

from myCode.skill_executor import SkillExecutor
from config_controller import controller
from myCode.my_env.multi_object_lift import MultiObjectLift

app = FastAPI()

# ----------------------------
# Step 1: Define data models
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
    environment: Dict[str, List[ObjectSpec]]
    available_functions: Dict[str, FunctionSpec]
    examples: List[dict] = []
    instructions: str = ""

# ----------------------------
# Step 2: Prompt construction and LLM interaction
# ----------------------------
def construct_prompt(payload: PromptInput, command: str, mode: str = "chain", initial_command: str = None) -> str:
    obj_descriptions = "\n".join([
        f"- {obj.name}: shape={obj.shape}, color={obj.color}, position={obj.position}, size={obj.size}"
        for obj in payload.environment.get("objects", [])
    ])

    func_descriptions = "\n".join([
        f"- {name}({', '.join(spec.params)}) -> modes: {spec.modes}"
        for name, spec in payload.available_functions.items()
    ])

    example_blocks = ""
    if payload.examples:
        formatted_examples = []
        for ex in payload.examples:
            ex_task = ex.get("task", "")
            ex_plan = ex.get("plan", [])
            plan_str = "\n  ".join([str(tuple(step)) for step in ex_plan])
            formatted_examples.append(f'Example Task: "{ex_task}"\n  Plan:\n  {plan_str}')
        example_blocks = "\n\n" + "\n\n".join(formatted_examples)

    if mode == "override" or not initial_command:
        task_instruction = f"Now, generate a symbolic plan for this task:\n{command}"
    else:
        task_instruction = f"""Now, generate a symbolic plan for the original task, 
                            modified by the following instruction:
                            Original Task: {initial_command}
                            Follow-up Instruction: {command}"""

    prompt = f"""
    You are a robot planning assistant. Your job is to interpret natural language commands and generate symbolic manipulation plans based on the environment, using available robot functions. You reason geometrically, use object positions, and compute targets when needed (e.g., for ambiguous spatial tasks). You always explain your logic clearly before giving the symbolic plan.

    Environment Objects:
    {obj_descriptions}

    Available Functions:
    {func_descriptions}

    Instructions:
    {payload.instructions.strip()}

    {example_blocks}

    {task_instruction}

    Note:
    For tasks like “move close to” or “put together” 
    - First compute the direction vector between the two objects.
    - Then choose an interpolation factor α ∈ [0, 1] and move one object along that direction.
    - Before finalizing the plan, **check whether the final distance between the two objects is at least 0.06**.
    - If the α results in a distance < 0.06 (too close / overlap), **reduce α** accordingly.
    - You must explain each of these steps before outputting the symbolic plan.

    For tasks like “separate A and B”:
    - Compute the direction from A to B.
    - Reverse the direction to increase separation.
    - Choose a reasonable interpolation factor α (e.g., 1.0 for full length, >1.0 to increase it).
    - There is no strict constraint on separation distance, but you should ensure the final distance is larger than before.
    - Your choice of α may be revised later based on scene layout or user feedback.

    First, verify if the command is valid based on the current environment objects.

    If the command references any objects that do not exist, or violates spatial constraints, explain the issue clearly.

    If the task is valid:
    - Explain how the symbolic plan is generated, using positions, geometry, and available functions.
    - Then return the symbolic plan.

    If you're unsure how to proceed, ask the user for clarification.

    Respond in the following format:

    Explanation:
    <your reasoning>

    Symbolic Plan:
    <valid Python list of tuples>

    """.strip()
    print("Environment objects passed to LLM from the base prompt:")
    for obj in payload.environment.get("objects", []):
        print(vars(obj))

    return prompt

def call_llm(prompt: str):
    openai.api_key = os.getenv("OPENAI_API_KEY")

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.5
    )

    content = response["choices"][0]["message"]["content"]
    # Debugging output
    print("Raw LLM response: ", content)

    try:
        explanation = extract_block(content, "Explanation:")
        plan_str = extract_block(content, "Symbolic Plan:")
        symbolic_plan = ast.literal_eval(plan_str.strip())
    except Exception as e:
        raise RuntimeError(f"Failed to parse LLM response: {e}")

    return {
        "explanation": explanation,
        "symbolic_plan": symbolic_plan
    }

def extract_block(text, header):
    lines = text.splitlines()
    found = False
    block = []
    for line in lines:
        if found:
            if line.strip() == "":
                break
            block.append(line)
        elif line.strip().startswith(header):
            found = True
    return "\n".join(block).strip()

# ----------------------------
# Step 3: Simulator Wrapper
# ----------------------------
class SimWrapper:
    def __init__(self):
        self.env = MultiObjectLift(
            robots="Panda",
            controller_configs=controller,
            has_renderer=True
        )
        self.env.reset()

        for body_name in self.env.sim.model.body_names:
            if "cube_main" in body_name:
                raise ValueError(f"The default cube '{body_name}' still exists in the environment. Please remove it before running the simulation.")
        self.executor = SkillExecutor(self.env)

    def get_current_state(self):
        return self.executor.get_all_object_descriptions()

    def execute_plan(self, plan):
        print("Executing:", plan)
        self.executor.execute_plan(plan)

    def reset_robot(self):
        self.executor.reset_robot_only()

def check_scene_consistency(current_objs: List[ObjectSpec], expected_objs: List[ObjectSpec], threshold=0.01):
    """
    Raises ValueError if the current scene deviates too much from the prompt baseline.
    Compares only positions of objects with the same name.
    If expected position is missing z, it infers based on shape.
    """
    mismatches = []
    expected_dict = {obj.name: obj for obj in expected_objs}

    for obj in current_objs:
        if obj.name in expected_dict:
            expected = expected_dict[obj.name]
            cur_pos = np.array(obj.position[:3])  # Use actual 3D from simulator

            # --- Handle expected position autofill ---
            table_height = 0.8
            exp_pos_raw = expected.position
            if len(exp_pos_raw) == 2:
                z_val = table_height + 0.0125 # if expected.shape == "cube" else 0.25
                exp_pos = np.array([*exp_pos_raw, z_val])
            else:
                exp_pos = np.array(exp_pos_raw[:3])

            dist = np.linalg.norm(exp_pos - cur_pos)
            if dist > threshold:
                mismatches.append((obj.name, dist, cur_pos.tolist(), exp_pos.tolist()))
    
    if mismatches:
        details = "\n".join([
            f"{name}: distance={dist:.3f}, current={cur}, expected={exp}"
            for name, dist, cur, exp in mismatches
        ])
        raise ValueError(f"🚨 Scene mismatch detected:\n{details}")
    else:
        print("✅ Scene consistency check passed.")


def load_default_prompt(path="prompt.json") -> PromptInput:
    with open(path, "r") as f:
        data = json.load(f)
    return PromptInput(**data)

# ----------------------------
# Step 4: FastAPI app and session
# ----------------------------
sim = SimWrapper()
SESSION = {
    "base_prompt": None,
    "initial_command": None,
    "history": []
}

async def lifespan(app: FastAPI):
    prompt_path = Path('/home/jingyang/robosuite/myCode/objective1_prompt_with_better_lift_logics.json')
    try:
        base_prompt = load_default_prompt(prompt_path)
        SESSION["base_prompt"] = base_prompt
        # Load real scene state
        raw_objects = sim.get_current_state()
        scene_objects = [ObjectSpec(**obj) for obj in raw_objects.values()]
        expected_objects = base_prompt.environment.get("objects", [])
        # Compare to expected prompt state
        print("🔍 Checking scene consistency at startup...")
        check_scene_consistency(
            current_objs=scene_objects,
            expected_objs=expected_objects, # loaded from prompt
            threshold=0.05  # or 0.01 if stricter
        )


        SESSION["initial_command"] = None
        SESSION["history"] = []
        print("✅ Loaded pre-defined.json at startup.")
    except Exception as e:
        print(f"⚠️ Failed to load prompt: {e}")
    yield

app = FastAPI(lifespan=lifespan)

@app.post("/init_session")
def init_session(payload: PromptInput):
    SESSION["base_prompt"] = payload
    SESSION["initial_command"] = None
    SESSION["history"] = []
    return {"status": "Session initialized"}

@app.post("/chat_step")
def chat_step(command: str, mode: str = None):
    if not SESSION["base_prompt"]:
        return {"error": "Session not initialized. Use /init_session first."}

    payload = SESSION["base_prompt"]

    if SESSION["initial_command"] is None:
        mode = "override"
        SESSION["initial_command"] = command
        print("🔄 Initial command set:", command)
    else:
        mode = mode or "chain"

    SESSION["history"].append((mode, command))

    raw_objects = sim.get_current_state()
    object_specs = [ObjectSpec(**obj) for obj in raw_objects.values()]
    payload.environment["objects"] = object_specs
    # ✅ Check scene match
    try:
        check_scene_consistency(
            current_objs=object_specs,
            expected_objs=payload.environment["objects"],  # <-- Already ObjectSpec instances
            threshold=0.05
        )
    except ValueError as e:
        print(str(e))
        return {"error": "Scene mismatch", "details": str(e)}
    
    prompt = construct_prompt(payload, command, mode, SESSION["initial_command"])

    try:
        result = call_llm(prompt)
        print("🔍 LLM Explanation:\n", result["explanation"])
        print("📋 Symbolic Plan:\n", result["symbolic_plan"])
    except Exception as e:
        return {"error": f"LLM failed: {e}"}

    sim.execute_plan(result["symbolic_plan"])

    return {
        "mode": mode,
        "command": command,
        "explanation": result["explanation"],
        "symbolic_plan": result["symbolic_plan"],
        "current_objects": raw_objects
    }

@app.post("/reset_scene_and_robot")
def reset_scene_and_robot():
    global sim
    try:
        sim.executor.idle()  # stop any motion safely
        sim.env.close()  # close rendering
        sim = SimWrapper()  # reinitialize everything
        SESSION["initial_command"] = None
        SESSION["history"] = []
        print("✅ Full reset complete")
        return {"status": "Scene and robot reset"}
    except Exception as e:
        print(f"⚠️ Reset crashed: {e}")
        return {"error": str(e)}

@app.post("/reset_robot_only")
def reset_robot_only():
    sim.reset_robot()
    SESSION["initial_command"] = None
    return {"status": "Robot reset, scene preserved"}

@app.get("/")
def read_root():
    return {"message": "Interactive Planning API at http://127.0.0.1:8000/docs. Use /init_session then /chat_step."}
