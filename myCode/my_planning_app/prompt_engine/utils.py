# prompt_engine/utils.py

import json
import re
from pathlib import Path
from datetime import datetime
import ast

def extract_symbolic_plan_block(text: str, header: str) -> str:
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

def extract_symbolic_plan_block(text: str, header: str = "Symbolic Plan") -> str:
    lines = text.splitlines()
    found = False
    block = []
    for line in lines:
        if not found:
            if line.strip().lower().startswith(header.lower()):
                found = True
            continue
        # Collect lines that look like list items
        if "[" in line and "]" in line:
            # Strip trailing commas or whitespace
            clean_line = line.strip().rstrip(",")
            block.append(clean_line)
    # Wrap in brackets to make a valid Python list
    return "[\n" + ",\n".join(block) + "\n]"

def extract_waypoints(response: str):
    """
    Extracts the first list of 2D waypoints from the raw LLM response.
    """
    # Match something that looks like [[x, y], [x, y], ...]
    match = re.search(r'\[\s*\[.*?\]\s*\]', response, re.DOTALL)
    
    if match:
        try:
            waypoints = ast.literal_eval(match.group())
            if isinstance(waypoints, list) and all(isinstance(p, list) and len(p) == 2 for p in waypoints):
                return waypoints
        except Exception as e:
            print(f"Failed to parse waypoint list: {e}")
    
    return None

def extract_explanation_block(text: str) -> str:
    lines = text.splitlines()
    found = False
    block = []
    for line in lines:
        stripped = line.strip()
        if found:
            if stripped.lower().startswith("symbolic plan"):
                break
            block.append(line)
        elif stripped.lower().startswith("explanation:"):
            found = True
            # Include the "Explanation:" line itself
            block.append(line)
    return "\n".join(block).strip()

def format_examples(example_list):
    if not example_list:
        return ""
    
    formatted = []
    for ex in example_list:
        ex_task = ex.get("task", "")
        ex_plan = ex.get("plan", [])
        ex_explanation = ex.get("explanation", "")

        plan_str = "\n    ".join([str(tuple(step)) for step in ex_plan])
        block = f'Example Task: "{ex_task}"\n  Plan:\n    {plan_str}'

        if ex_explanation:
            indented_explanation = ex_explanation.replace("\n", "\n    ")
            block += "\n  Explanation:\n    " + indented_explanation

        formatted.append(block)

    return "\n\n" + "\n\n".join(formatted)

def format_trajectory_examples(example_list):
    if not example_list:
        return ""
    
    formatted = []
    for ex in example_list:
        ex_task = ex.get("task", "")
        ex_points = [ex.get("points of trajectory", [])]  # renamed from 'plan'
        ex_explanation = ex.get("explanation", "")

        # Format the list of 2D waypoints
        points_str = "\n    ".join([str(point) for point in ex_points])
        block = f'Example Task: "{ex_task}"\n  Trajectory Points:\n    {points_str}'

        if ex_explanation:
            indented_explanation = ex_explanation.replace("\n", "\n    ")
            block += "\n  Explanation:\n    " + indented_explanation

        formatted.append(block)

    return "\n\n" + "\n\n".join(formatted)

def log_experiment_entry(entry: dict, filename_prefix: str = "log"):
    LOG_DIR = Path.home() / "robosuite" / "myCode" / "my_planning_app" / "logs"
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    initial_command = entry.get("initial_command", "unknown")
    filename = f"{filename_prefix}_{initial_command}_{timestamp}.json"
    file_path = LOG_DIR / filename

    with open(file_path, "w") as f:
        json.dump(entry, f, indent=2)

    print(f"✅ Experiment log saved to: {file_path}")

def log_task_summary(SESSION: dict):
    if not SESSION["current_task_logs"]:
        return

    initial_cmd = SESSION["initial_command"]
    initial_cmd_save_format = '_'.join(initial_cmd.split())
    task_type = SESSION.get("task_type", "unknown")
    full_log = {
        "initial_command": initial_cmd_save_format,
        "timestamp": datetime.now().isoformat(),
        "steps": SESSION["current_task_logs"]
    }

    log_experiment_entry(full_log, filename_prefix=task_type)

    # reset task buffer
    SESSION["current_task_logs"] = []
    SESSION["initial_command"] = None