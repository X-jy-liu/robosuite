import requests
import time
import argparse
import importlib

BASE_URL = "http://localhost:8000"

def run_ambiguous_experiment(exp_config):
    print(f"\n🤔 [Ambiguous] Starting experiment: {exp_config['name']}")
    # Use the first string from the lists
    payload = {
        "commands": exp_config["commands"],
        "task_type": exp_config["task_type"],
        "regenerate_scene": exp_config["regenerate_scene"],
        "regenerate_dots": exp_config["regenerate_dots"],
        "ambiguous_effects": exp_config.get("ambiguous_effects", False),
    }
    res = requests.post(f"{BASE_URL}/run_ambiguous_experiment", json=payload)
    if res.status_code != 200:
        print(f"❌ Error: {res.text}")
    else:
        print(f"✅ Finished: {exp_config['name']}")
    time.sleep(1)

def build_config_path(scene_idx: str) -> str:
    return f"logs.scene_{scene_idx}.scene_{scene_idx}_experiments_config"

def load_experiments(config_path: str):
    try:
        module = importlib.import_module(config_path)
        return getattr(module, 'experiments')  # 'experiments' must be lowercase in your .py file
    except (ModuleNotFoundError, AttributeError) as e:
        raise ImportError(f"Cannot load experiments from '{config_path}': {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run experiments for a given scene index.")
    parser.add_argument(
        "--scene",
        required=True,
        help="Scene index only takes '06' or '07', which maps to logs.scene_XX.scene_XX_experiments_config"
    )
    args = parser.parse_args()
    if args.scene not in ["06", "07"]:
        raise ValueError("Scene index must be '06' or '07'.")
    config_path = build_config_path(args.scene)
    experiments = load_experiments(config_path)

    for exp in experiments:
        run_ambiguous_experiment(exp)