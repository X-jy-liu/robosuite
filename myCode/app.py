from fastapi import FastAPI
from pydantic import BaseModel
from typing import List, Dict
import openai
import os
import ast  # for safe evaluation of string output from LLM

app = FastAPI()

# ----------------------------
# Step 3: Define input format
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

EXAMPLE_PLAN = """
    Example:
    Given this task: "Stack the red cube on the blue cube."

    Plan:
    [
    ("move", "red_cube", "above"),
    ("gripper_open",),
    ("move", "red_cube", "contact"),
    ("gripper_close",),
    ("lift", "red_cube", "above"),
    ("move", "blue_cube", "above"),
    ("gripper_open",)
    ]
    """

# -----------------------------------
# Step 4: Build prompt from JSON input
# -----------------------------------
def construct_prompt(payload: PromptInput) -> str:
    obj_descriptions = "\n".join([str(obj.model_dump()) for obj in payload.environment["objects"]])
    func_descriptions = "\n".join([
        f"{name}({', '.join(spec.params)}) -> modes: {spec.modes}"
        for name, spec in payload.available_functions.items()
    ])

    prompt = f"""
        Task: {payload.task}

        Environment Objects:
        {obj_descriptions}

        Available Functions:
        {func_descriptions}

        Instructions:
        Return a symbolic plan using only the available functions. Use correct syntax and function signatures. Respond ONLY with a valid Python list of tuples. Do NOT include 'Plan:', 'Answer:', or any explanation text.

        {EXAMPLE_PLAN}
        Now, generate a symbolic plan for this task:
        {payload.task}
        """
    return prompt

# -----------------------------------
# Step 5: Query GPT-4 or similar LLM
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

# -----------------------------
# Step 6: API route definition
# -----------------------------
@app.post("/generate_plan")
def generate_plan(payload: PromptInput):
    prompt = construct_prompt(payload)
    plan = call_llm(prompt)
    return plan

# visit the following URL to test the API: http://localhost:8000/docs