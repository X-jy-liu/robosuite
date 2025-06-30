import json
from parser_prompt_builder import build_prompt
from call_llm_parser import call_llm_parser
from log_reader import read_experiment_log
from pathlib import Path
import os

# Load JSON files
env_path = Path.home()/"robosuite"/"myCode"/"my_planning_app"/"prompts"/"env_and_func.json"
with open(env_path, "r") as f:
    env_data = json.load(f)

dots_path = Path.home()/"robosuite"/"myCode"/"my_planning_app"/"prompts"/"generated_dots.json"
with open(dots_path, "r") as f:
    dots_data = json.load(f)

# response save path
response_save_dir = Path.home()/"robosuite"/"myCode"/"my_planning_app"/"logs"
os.makedirs(response_save_dir, exist_ok=True)

# Extract required sections
objects = env_data["environment"]["objects"]
reference_points = dots_data["reference_points"]

# extract command, tasks_type, init_obj_specs, and obj_specs_history
log_dir = Path.home()/"robosuite"/"myCode"/"my_planning_app"/"logs"
log_paths = list(log_dir.glob("*.json")) # read all log .json files within the log_dir
for i, log_path in enumerate(log_paths):
    print(f"Processing log file {i+1}/{len(log_paths)} ...")
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

    llm_repsonse = call_llm_parser(full_prompt)
    # Save the response
    response_save_path = response_save_dir / f"{log_name}_parsed.json"
    with open(response_save_path, "w") as f:
        json.dump(llm_repsonse, f, indent=4)
    
    print(f"Finished {i+1}/{len(log_paths)}! Response saved to {response_save_path}")
    

    # if task_type == "simple":
    #     return simple_evaluator.evaluate(parsed, start_state, end_state, trajectories)
    # elif task_type == "ambiguous":
    #     return ambiguous_evaluator.evaluate(parsed, start_state, end_state, trajectories)
    # elif task_type == "trajectory":
    #     return trajectory_evaluator.evaluate(parsed, start_state, end_state, trajectories)

if __name__ == "__main__":
    print(f"Responses saved in: {response_save_dir}")
