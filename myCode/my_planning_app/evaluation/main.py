import json
from parser_prompt_builder import build_prompt
from call_llm_parser import call_llm_parser
from log_reader import read_experiment_log
from pathlib import Path
import os
from evaluator import Evaluator
from mapping_generator import obj_name_mapping
import argparse


def process_scene(scene_index: str, phase_number: str):
    base_path = Path.home() / "robosuite" / "myCode" / "my_planning_app"

    # Load environment and dots
    scene_dir = base_path / "logs" / f"scene_{scene_index}"
    if not scene_dir.exists():
        raise FileNotFoundError(f"Scene directory {scene_dir} does not exist.")
    env_path = scene_dir / "env_and_func.json"
    dots_path = scene_dir / "generated_dots.json"
    with open(env_path, "r") as f:
        env_data = json.load(f)
    with open(dots_path, "r") as f:
        dots_data = json.load(f)

    obj_list_dict = env_data["environment"]["objects"]
    assert isinstance(obj_list_dict, list), "Expected 'objects' to be a list of dictionaries."
    assert len(obj_list_dict) == 5, "Expected 'objects' to contain exactly 5 items."
    obj_mapping = obj_name_mapping(obj_list_dict)
    ref_pnt_mapping = dots_data["reference_points"]

    # Construct log and save paths
    log_dir = base_path / "logs" / f"scene_{scene_index}" / "experiment_logs" / f"phase_{phase_number}"
    response_save_dir = base_path / "logs" / f"scene_{scene_index}" / "evaluation_logs" / f"phase_{phase_number}"
    os.makedirs(response_save_dir, exist_ok=True)

    assert log_dir.exists(), f"Log directory {log_dir} does not exist."
    print(f"Processing logs from {log_dir}...")
    log_paths = sorted(list(log_dir.glob("*.json")))

    objects = env_data["environment"]["objects"]
    reference_points = dots_data["reference_points"]

    for i, log_path in enumerate(log_paths):
        print(f"\nProcessing log file {i + 1}/{len(log_paths)}...\n   \"{log_path.stem}\"\n")
        log_name = log_path.stem

        initial_command, task_type, init_obj, obj_pos_history = read_experiment_log(log_path)

        full_prompt = build_prompt(
            command=initial_command,
            task_type=task_type,
            environment_objects=objects,
            reference_points=reference_points,
            include_examples=True
        )

        llm_evaluation = call_llm_parser(full_prompt)
        assert isinstance(llm_evaluation, dict), "Expected LLM response to be a dictionary."

        print(f"Command: {initial_command}")
        print(f"Task Type: {task_type}")
        print(f"interested_object: {llm_evaluation['interested_object']}")
        print(f"success_criteria: {llm_evaluation['success_criteria']}")

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
            print(f"::Basic evaluation success: {if_success}")
        elif task_type == "ambiguous":
            if_success = evaluator.ambiguous_evaluate()
            print(f"::Ambiguous evaluation success: {if_success}")
        elif task_type == "trajectory":
            if_success = evaluator.trajectory_evaluate()
            print(f"::Trajectory evaluation success: {if_success}")
        else:
            raise ValueError(f"Unknown task type: {task_type}")

        llm_evaluation["success_status"] = "success" if if_success else "failure"

        response_save_path = response_save_dir / f"{log_name}_evaluated.json"
        with open(response_save_path, "w") as f:
            json.dump(llm_evaluation, f, indent=4)

        print(f"Finished {i + 1}/{len(log_paths)} ...")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate logs from a given scene.")
    parser.add_argument("--scene", type=str, required=True, help="Scene index, e.g., 01 or 02")
    parser.add_argument("--phase", type=str, required=True, help="Phase of the scene, e.g., 1, 2, 3")
    args = parser.parse_args()
    if args.phase not in ["1", "2", "3"]:
        raise ValueError(f"Invalid phase: {args.phase}. Expected one of: 1, 2, 3")

    process_scene(args.scene, args.phase)