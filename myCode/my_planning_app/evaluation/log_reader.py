import json
from typing import Tuple, Dict, List, Any


def read_experiment_log(log_path: str) -> Tuple[str, str, Dict[str, Any], List[Dict[str, Any]]]:
    """
    Reads an experiment log JSON and extracts:
      - initial command string
      - task type
      - initial object info
      - object position history

    Args:
        log_path: Path to the JSON log file.

    Returns:
        Tuple containing:
            - initial_command (str)
            - task_type (str)
            - init_obj (dict)
            - obj_pos_history (list of dicts)
    """
    with open(log_path, 'r') as f:
        log_data = json.load(f)

    # Top-level initial command
    initial_command = log_data.get("initial_command", "").replace("_"," ")

    # Assuming first step is sufficient for task_type and state tracking
    step = log_data.get("steps", [{}])[0]

    task_type = step.get("task_type", "")
    init_obj = step.get("init_obj", {})
    obj_pos_history = step.get("obj_pos_history", [])

    return initial_command, task_type, init_obj, obj_pos_history

if __name__ == "__main__":
    path  = '/home/jingyang/robosuite/myCode/my_planning_app/logs/trajectory_move_the_blue_cube_to_point_4_then_to_point_2_20250629_220106.json'
    initial_command, task_type, init_obj, obj_pos_history = read_experiment_log(path)
    print(f"Initial Command: {initial_command}")
    print(f"Task Type: {task_type}")
    print(f"Initial Object State: {init_obj}")
    print(f"Object Position History: {obj_pos_history}[0:5]")  # Print first 5 entries for brevity
