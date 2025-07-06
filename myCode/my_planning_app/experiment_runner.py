import requests
import time
import argparse
import importlib

BASE_URL = "http://localhost:8000"

def run_experiment(exp_config):
    print(f"\n🚀 Starting experiment: {exp_config['name']}")
    payload = {
        "regenerate_scene": exp_config["regenerate_scene"],
        "regenerate_dots": exp_config["regenerate_dots"],
        "task_type": exp_config["task_type"],
        "commands": exp_config["commands"],
        "mode": "none"
    }

    res = requests.post(f"{BASE_URL}/run_full_experiment", json=payload)
    if res.status_code != 200:
        print(f"❌ Error: {res.text}")
    else:
        print(f"✅ Finished: {exp_config['name']}")

    time.sleep(1)

def build_config_path(scene_idx: str) -> str:
    """
    Given a scene index like '01', return the full module path.
    """
    return f"logs.scene_{scene_idx}.scene_{scene_idx}_experiments_config"

def load_experiments(config_path: str):
    try:
        module = importlib.import_module(config_path)
        return getattr(module, 'EXPERIMENTS')
    except (ModuleNotFoundError, AttributeError) as e:
        raise ImportError(f"Cannot load EXPERIMENTS from '{config_path}': {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run experiments for a given scene index.")
    parser.add_argument(
        "--scene",
        required=True,
        help="Scene index, e.g., '01', '02', ..., which maps to logs.scene_XX.scene_XX_experiments_config"
    )
    args = parser.parse_args()

    config_path = build_config_path(args.scene)
    experiments = load_experiments(config_path)

    for exp in experiments:
        run_experiment(exp)
