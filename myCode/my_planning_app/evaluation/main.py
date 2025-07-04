import json
from parser_prompt_builder import build_prompt
from call_llm_parser import call_llm_parser
from log_reader import read_experiment_log
from pathlib import Path
import os
from evaluator import Evaluator
from mapping_generator import obj_name_mapping

# Load JSON files
env_path = Path.home()/"robosuite"/"myCode"/"my_planning_app"/"prompts"/"env_and_func.json"
with open(env_path, "r") as f:
    env_data = json.load(f)

obj_list_dict = env_data["environment"]["objects"]
assert isinstance(obj_list_dict, list), "Expected 'objects' to be a list of dictionaries."
assert len(obj_list_dict) ==5, "Expected 'objects' to contain exactly 5 items."
obj_mapping = obj_name_mapping(obj_list_dict)

dots_path = Path.home()/"robosuite"/"myCode"/"my_planning_app"/"prompts"/"generated_dots.json"
with open(dots_path, "r") as f:
    dots_data = json.load(f)
ref_pnt_mapping = dots_data["reference_points"]

# response save path
response_save_dir = Path.home()/"robosuite"/"myCode"/"my_planning_app"/"logs"/"scene_01"/"llm_evaluated_responses"
os.makedirs(response_save_dir, exist_ok=True)

# Extract required sections
objects = env_data["environment"]["objects"]
reference_points = dots_data["reference_points"]

# extract command, tasks_type, init_obj_specs, and obj_specs_history
log_dir = Path.home()/"robosuite"/"myCode"/"my_planning_app"/"logs"/"scene_01"/"experiments_logging"
assert log_dir.exists(), f"Log directory {log_dir} does not exist."
log_paths = list(log_dir.glob("*.json")) # read all log .json files within the log_dir
for i, log_path in enumerate(log_paths):
    print(f"\nProcessing log file {i+1}/{len(log_paths)}...\n   \"{log_path.stem}\"\n")
    log_name = log_path.stem  # Get the name of the log file without extension
    # read experiment logs
    initial_command, task_type, init_obj, obj_pos_history = read_experiment_log(log_path)

    # Build prompt
    full_prompt = build_prompt(
        command=initial_command,
        task_type=task_type,
        environment_objects=objects,
        reference_points=reference_points,
        include_examples=True  # set to False if you just want the env + command
    )

    llm_evaluation = call_llm_parser(full_prompt) # keys in llm_evaluation: dict_keys(['task_type', 'action', 'interested_object', 'success_criteria', 'explanation'])
    assert isinstance(llm_evaluation, dict), "Expected LLM response to be a dictionary."
    print(f"Command: {initial_command}")
    print(f"Task Type: {task_type}")
    print(f"interested_object: {llm_evaluation['interested_object']}")
    print(f"success_criteria: {llm_evaluation['success_criteria']}")

    # create the Evaluator instance
    evaluator = Evaluator(
        task_type=task_type,
        action=llm_evaluation.get("action", ""),
        interested_obj=llm_evaluation.get("interested_object", ""),
        success_criteria=llm_evaluation.get("success_criteria", None),
        init_obj_specs=init_obj,
        obj_specs_history=obj_pos_history,
        obj_mapping=obj_mapping,
        ref_pnt_mapping=ref_pnt_mapping
    )

    if task_type == "basic":
        if_success = evaluator.basic_evaluate()
        llm_evaluation["success_status"] = "success" if if_success else "failure"

    elif task_type == "ambiguous":
        if_success = evaluator.ambiguous_evaluate()
        llm_evaluation["success_status"] = "success" if if_success else "failure"
    elif task_type == "trajectory":
        if_success = evaluator.trajectory_evaluate()
        llm_evaluation["success_status"] = "success" if if_success else "failure"
    else:
        raise ValueError(f"Unknown task type: {task_type}")

    # Save the response
    response_save_path = response_save_dir / f"{log_name}_evaluated.json"
    with open(response_save_path, "w") as f:
        json.dump(llm_evaluation, f, indent=4)
    
    print(f"Finished {i+1}/{len(log_paths)}!\n  Response saved to {response_save_path}\n")