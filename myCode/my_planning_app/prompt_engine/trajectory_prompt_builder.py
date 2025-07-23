import json
from pathlib import Path
from typing import Optional

from .utils import format_trajectory_examples

# Path config (adjust if needed)
# load home directory
HOME_DIR = Path.home()
# Define the directory where prompt files are stored
base_prompt_dir = HOME_DIR / "robosuite" / "myCode" / "my_planning_app" / "prompts"

def load_json(file_dir: Path, filename: str):
    with (file_dir / filename).open() as f:
        return json.load(f)

# Core: build the full prompt
def construct_trajectory_prompt(command: str,
                     task_type: str,
                     mode: str,
                     scene_dir: Path) -> str:
    if task_type != "trajectory":
        raise ValueError(f"Unsupported task type: {task_type}. Only 'trajectory' is supported.")
    # 1. Load base environment and function definitions
    base_data = load_json(scene_dir, "env_and_func.json")
    scene_dots_data = load_json(scene_dir, "generated_dots.json")["reference_points"]
    environment = base_data["environment"]
    available_functions = base_data["available_functions"]

    # 2. Load shared instruction text
    instruction_block = """\
    For trajectory tasks:
    - First, write an explanation of how the waypoints were selected.
    - Then return only a list of 2D waypoints in the form [[x1, y1], [x2, y2], ..., [xn, yn]].
    - Do not return symbolic actions or function calls (e.g., 'gripper_move').
    - The waypoints should follow the intended path described in the task.
    - If the task includes intermediate points (e.g., 'via point_2'), include those coordinates.
    - Do not include Z coordinates — all points are assumed to be on the same plane.
    - Format your response exactly as follows:

    explanation:<your reasoning>
    trajectory_points:[[x1, y1], [x2, y2], ..., [xn, yn]]
    """


    # 3. Load task-specific examples
    example_data = load_json(base_prompt_dir, "trajectory.json")
    examples = example_data["examples"]

    example_dots_data = load_json(base_prompt_dir, "dots_example.json")["reference_points"]
    example_dots_descriptions = "\n".join([
        f"{name}: {coord}" for name, coord in example_dots_data.items()
    ])
    example_dots_block = f"\n\nReference Points:\n{example_dots_descriptions}"


    example_data = load_json(base_prompt_dir, f"{task_type}.json")
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
    scene_config_example = load_json(base_prompt_dir, "scene_config_example.json")["environment"]
    scene_config = "\n".join([
        f"{obj['name']}: shape={obj['shape']}, color={obj['color']}, position={obj['position']}, size={obj['size']}"
        for obj in scene_config_example.get("objects", [])
    ])

    example_blocks = format_trajectory_examples(examples)

    # 5. Construct task instruction
    if mode == "chain" and task_type == "trajectory":
        raise ValueError("Chaining mode is not supported for trajectory tasks for now.")
    
    task_instruction = f"Now, generate a symbolic plan for this task:\n{command}"

    prompt = f"""
You are a symbolic planner for a robot arm. You must generate a collision-free symbolic plan for moving objects in a cluttered scene.

Each object has a position and a radius of 0.05 meters (size = 0.05). Collisions must be avoided by checking that all straight-line path segments between waypoints are at least 0.07 meters away from any object center.

Available Functions:
{func_descriptions}

Instructions:
{instruction_block}

Example Scene and reference points:
{scene_config}
{example_dots_block}

Example Tasks and points of trajectory with explanations:
{example_blocks}

Current Scene and reference points (if applicable):
{obj_descriptions}
{scene_dots_block}

{task_instruction}
""".strip()

    return prompt

if __name__ == "__main__":
    # Example usage
    command = "move blue cube to point 5 via point 2 without collision"
    task_type = "trajectory"  # or "manipulation", "navigation", etc.
    prompt = construct_trajectory_prompt(command=command, task_type=task_type, mode="override")
    print(prompt)