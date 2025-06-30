import json
from parser_prompt_builder import build_prompt
from call_llm_parser import call_llm_parser
from log_reader import read_experiment_log
from pathlib import Path

# Load JSON files
env_path = Path.home()/"robosuite"/"myCode"/"my_planning_app"/"prompts"/"env_and_func.json"
with open(env_path, "r") as f:
    env_data = json.load(f)

dots_path = Path.home()/"robosuite"/"myCode"/"my_planning_app"/"prompts"/"generated_dots.json"
with open(dots_path, "r") as f:
    dots_data = json.load(f)

# Extract required sections
objects = env_data["environment"]["objects"]
reference_points = dots_data["reference_points"]

# extract command, tasks_type, init_obj_specs, and obj_specs_history
log_path = Path.home()/"robosuite"/"myCode"/"my_planning_app"/"logs"/"trajectory_move_the_blue_cube_to_point_4_then_to_point_2_20250629_220106.json"

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
print(f"LLM Response:\n {llm_repsonse}")

    # if task_type == "simple":
    #     return simple_evaluator.evaluate(parsed, start_state, end_state, trajectories)
    # elif task_type == "ambiguous":
    #     return ambiguous_evaluator.evaluate(parsed, start_state, end_state, trajectories)
    # elif task_type == "trajectory":
    #     return trajectory_evaluator.evaluate(parsed, start_state, end_state, trajectories)
