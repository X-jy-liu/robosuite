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
    examples: List[dict] = []
    instructions: str = ""


# -----------------------------------
# Step 4: Build prompt from JSON input
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