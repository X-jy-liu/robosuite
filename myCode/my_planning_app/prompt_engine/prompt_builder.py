import json
from pathlib import Path
from typing import Optional

from .utils import format_examples

# Path config (adjust if needed)
# load home directory
HOME_DIR = Path.home()
# Define the directory where prompt files are stored
PROMPT_DIR = HOME_DIR / "robosuite" / "myCode" / "my_planning_app" / "prompts"

def load_json(filename: str):
    with (PROMPT_DIR / filename).open() as f:
        return json.load(f)

# Core: build the full prompt
def construct_prompt(command: str,
                     task_type: str,
                     mode: str = "chain",
                     initial_command: Optional[str] = None) -> str:
    # 1. Load base environment and function definitions
    base_data = load_json("env_and_func.json")
    scene_dots_data = load_json("generated_dots.json")["reference_points"]
    environment = base_data["environment"]
    available_functions = base_data["available_functions"]

    # 2. Load shared instruction text
    shared_instruction_data = load_json("shared_instruction.json")
    shared_instructions = shared_instruction_data["instructions"]

    # 3. Load task-specific examples
    if task_type == "trajectory":
        example_data = load_json("trajectory.json")
        examples = example_data["examples"]

        example_dots_data = load_json("dots_example.json")["reference_points"]
        example_dots_descriptions = "\n".join([
            f"{name}: {coord}" for name, coord in example_dots_data.items()
        ])
        example_dots_block = f"\n\nReference Points:\n{example_dots_descriptions}"
    else:
        example_data = load_json(f"{task_type}.json")
        examples = example_data["examples"]

    # 4. Build object and function descriptions
    obj_descriptions = "\n".join([
        f"{obj['name']}: shape={obj['shape']}, color={obj['color']}, position={obj['position']}, size={obj['size']}"
        for obj in environment.get("objects", [])
    ])
    scene_dots_descriptions = "\n".join([
            f"{name}: {coord}" for name, coord in scene_dots_data.items()
        ])
    scene_dots_block = f"\n\nReference Points:\n{scene_dots_descriptions}"

    func_descriptions = "\n\n".join([
    f"  {name}({', '.join(spec['params'])})\n"
    f"  Description: {spec['description']}\n"
    f"  Examples: {spec['examples']}"
    for name, spec in available_functions.items()
    ])

    # 4. Format scene configuration
    scene_config_example = load_json("scene_config_example.json")["environment"]
    scene_config = "\n".join([
        f"{obj['name']}: shape={obj['shape']}, color={obj['color']}, position={obj['position']}, size={obj['size']}"
        for obj in scene_config_example.get("objects", [])
    ])

    example_blocks = format_examples(examples)

    # 5. Construct task instruction
    if mode == "chain" and initial_command:
        task_instruction = (
            "Now, generate a symbolic plan for the original task, modified by the following instruction:\n"
            f"Original Task: {initial_command}\n"
            f"Follow-up Instruction: {command}"
        )
    else:
        task_instruction = f"Now, generate a symbolic plan for this task:\n{command}"

    # 6. Assemble final prompt
    instruction_block = "\n".join(shared_instructions) if isinstance(shared_instructions, list) else shared_instructions

    prompt = f"""
You are a robot planning assistant that generates symbolic plans based on high-level language commands.

Available Functions:
{func_descriptions}

Instructions:
{instruction_block}

Example Scene and Tasks:

Example Scene and reference points (if applicable):
{scene_config}
{example_dots_block if task_type == "trajectory" else ""}

Example Tasks with explained symbolic plans matching the scene:
{example_blocks}

Current Scene and reference points (if applicable):
{obj_descriptions}
{scene_dots_block if task_type == "trajectory" else ""}

{task_instruction}
""".strip()

    return prompt

if __name__ == "__main__":
    # Example usage
    command = "put the red cube and blue cube together"
    task_type = "trajectory"  # or "manipulation", "navigation", etc.
    prompt = construct_prompt(command=command, task_type=task_type)
    print(prompt)