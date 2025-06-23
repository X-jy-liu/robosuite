# prompt_engine/utils.py

import json
from pathlib import Path
from datetime import datetime

def extract_block(text: str, header: str) -> str:
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

def log_experiment_entry(entry: dict, filename_prefix: str = "log"):
    LOG_DIR = Path.home() / "robosuite" / "myCode" / "my_planning_app" / "logs"
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{filename_prefix}_{timestamp}.json"
    file_path = LOG_DIR / filename

    with open(file_path, "w") as f:
        json.dump(entry, f, indent=2)

    print(f"✅ Experiment log saved to: {file_path}")

def log_task_summary(SESSION: dict):
    if not SESSION["current_task_logs"]:
        return

    initial_cmd = SESSION["initial_command"]
    full_log = {
        "initial_command": initial_cmd,
        "timestamp": datetime.now().isoformat(),
        "steps": SESSION["current_task_logs"]
    }

    log_experiment_entry(full_log, filename_prefix="task")

    # reset task buffer
    SESSION["current_task_logs"] = []
    SESSION["initial_command"] = None