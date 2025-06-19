from fastapi import FastAPI
from pydantic import BaseModel
from typing import List, Dict
import openai
import os
import ast  # for safe evaluation of string output from LLM
import json
from pathlib import Path
from robosuite.environments.manipulation.lift import Lift
from myCode.skill_executor import SkillExecutor
from config_controller import controller
from robosuite.models.objects import BoxObject, CylinderObject
from robosuite.utils.mjcf_utils import add_to_dict
from myCode.multi_object_lift import MultiObjectLift
import numpy as np

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


# -----------------------------------
# Step 2: Prompt construction and LLM interaction
# -----------------------------------
def construct_prompt(payload: PromptInput, command: str, mode: str = "chain", initial_command: str = None) -> str:
    # Format environment
    obj_descriptions = "\n".join([
        f"- {obj.name}: shape={obj.shape}, color={obj.color}, position={obj.position}, size={obj.size}"
        for obj in payload.environment.get("objects", [])
    ])

    # Format functions
    func_descriptions = "\n".join([
        f"- {name}({', '.join(spec.params)}) -> modes: {spec.modes}"
        for name, spec in payload.available_functions.items()
    ])

    # Format examples
    example_blocks = ""
    if payload.examples:
        formatted_examples = []
        for ex in payload.examples:
            ex_task = ex.get("task", "")
            ex_plan = ex.get("plan", [])
            plan_str = "\n  ".join([str(tuple(step)) for step in ex_plan])
            formatted_examples.append(f'Example Task: "{ex_task}"\n  Plan:\n  {plan_str}')
        example_blocks = "\n\n" + "\n\n".join(formatted_examples)

    # Final instruction
    if mode == "override" or not initial_command:
        task_instruction = f"Now, generate a symbolic plan for this task:\n{command}"
    else:  # mode == "chain"
        task_instruction = f"""Now, generate a symbolic plan for the original task, 
modified by the following instruction:
Original Task: {initial_command}
Follow-up Instruction: {command}"""

    # Assemble the full prompt
    prompt = f"""
Environment Objects:
{obj_descriptions}

Available Functions:
{func_descriptions}

Instructions:
{payload.instructions.strip()}

{example_blocks}

{task_instruction}

First, verify if the command is valid based on the current environment objects.

If the command references any objects that do not exist, or violates spatial constraints, explain the issue clearly.

If possible, generate a corrected version of the command and proceed to create a symbolic plan for that.

Otherwise, return an empty plan (`[]`) and describe why no valid plan can be generated.

If you're unsure how to proceed, ask the user for clarification in your explanation.

Respond in the following format:

Explanation:
<your reasoning>

Corrected Command (if applicable):
<corrected command or same as input>

Symbolic Plan:
<valid Python list of tuples>

""".strip()

    return prompt

def call_llm(prompt: str):
    openai.api_key = os.getenv("OPENAI_API_KEY")

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.5
    )

    content = response["choices"][0]["message"]["content"]

    try:
        # Use simple heuristics to parse response
        explanation = extract_block(content, "Explanation:")
        corrected_command = extract_block(content, "Corrected Command:")
        plan_str = extract_block(content, "Symbolic Plan:")
        symbolic_plan = ast.literal_eval(plan_str.strip())
    except Exception as e:
        raise RuntimeError(f"Failed to parse LLM response: {e}")

    return {
        "explanation": explanation,
        "corrected_command": corrected_command,
        "symbolic_plan": symbolic_plan
    }

def extract_block(text, header):
    """Helper to extract the text after a header like 'Explanation:'"""
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

# -----------------------------------
# Step 4: Simulator and utils
# -----------------------------------

class SimWrapper:
    def __init__(self):
        self.env = MultiObjectLift(
            robots="Panda",
            controller_configs=controller,
            has_renderer=True  # Set to True only if needed
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

def load_default_prompt(path="prompt.json") -> PromptInput:
    with open(path, "r") as f:
        data = json.load(f)
    return PromptInput(**data)

# -----------------------------------
# Step 4: FastAPI lifespan management and app definition
# -----------------------------------

# Global state
sim = SimWrapper()
session = {"running": False}
SESSION = {
    "base_prompt": None,
    "initial_command": None,
    "history": []
}
log = []

async def lifespan(app: FastAPI):
    prompt_path = Path('/home/jingyang/robosuite/myCode/objective1_prompt_with_better_lift_logics.json')
    try:
        SESSION["base_prompt"] = load_default_prompt(prompt_path)
        SESSION["initial_command"] = None
        SESSION["history"] = []
        print("✅ Loaded pre-defined.json at startup.")
    except Exception as e:
        print(f"⚠️ Failed to load prompt.json: {e}")
    yield

app = FastAPI(lifespan=lifespan)

@app.post(
        "/init_session",
        description="Manually reinitialize the planning session. Only use this if you want to override the default tabletop setup loaded from prompt.json."
        )
def init_session(payload: PromptInput):
    SESSION["base_prompt"] = payload
    SESSION["initial_command"] = None  # reset
    SESSION["history"] = []
    return {"status": "Session initialized"}

@app.post("/chat_step")
def chat_step(command: str, mode: str = None):
    if not SESSION["base_prompt"]:
        return {"error": "Session not initialized. Use /init_session first."}

    payload = SESSION["base_prompt"]

    # Decide mode automatically if not explicitly given
    if SESSION["initial_command"] is None:
        mode = "override"
        SESSION["initial_command"] = command
    else:
        mode = mode or "chain"

    SESSION["history"].append((mode, command))

    # Update environment objects
    raw_state = sim.get_current_state()
    object_specs = [
        ObjectSpec(
            name=name,
            shape="cube" if "cube" in name else "cylinder",
            color="red",  # optionally update with real color detection
            position=pos.tolist(),
            size=0.025
        ) for name, pos in raw_state.items()
    ]
    payload.environment["objects"] = object_specs

    # Build prompt
    prompt = construct_prompt(payload, command, mode, SESSION["initial_command"])

    try:
        result = call_llm(prompt)
    except Exception as e:
        return {"error": f"LLM failed: {e}"}

    sim.execute_plan(result["symbolic_plan"])

    return {
        "mode": mode,
        "command": command,
        "corrected_command": result["corrected_command"],
        "explanation": result["explanation"],
        "symbolic_plan": result["symbolic_plan"],
        "current_objects": raw_state
}


@app.get("/")
def read_root():
    return {"message": "Interactive Planning API in http://127.0.0.1:8000/docs. Use /init_session then /chat_step."}
