import requests
import time
from datetime import datetime

BASE_URL = "http://localhost:8000"

EXPERIMENTS = [
    {
        "name": "Scene1_basic_1",
        "regenerate_scene": False,
        "regenerate_dots": False,
        "task_type": "basic",
        "commands": ["Lift the blue cube"],
        "mode": "override"
    },
    {
        "name": "Scene1_basic_2",
        "regenerate_scene": False,
        "regenerate_dots": False,
        "task_type": "basic",
        "commands": ["Move the green cube to [0.1,-0.1,0.825]"],
        "mode": "override"
    },
    {
        "name": "Scene1_basic_3",
        "regenerate_scene": False,
        "regenerate_dots": False,
        "task_type": "basic",
        "commands": ["Lift the red cylinder"],
        "mode": "override"
    },
    {
        "name": "scene1_ambiguous_1",
        "regenerate_scene": False,
        "regenerate_dots": False,
        "task_type": "ambiguous",
        "commands": ["put the red cubes together"],
        "mode": "override"
    },
    {
        "name": "scene1_ambiguous_2",
        "regenerate_scene": False,
        "regenerate_dots": False,
        "task_type": "ambiguous",
        "commands": ["increase the distance between the green cube and the blue cube"],
        "mode": "override"
    },
    {
        "name": "Scene1_trajectory_1",
        "regenerate_scene": False,
        "regenerate_dots": False,
        "task_type": "trajectory",
        "commands": ["move the blue cube to point 3, then point 1, then point 2"],
        "mode": "override"
    },
    {
        "name": "Scene1_trajectory_2",
        "regenerate_scene": False,
        "regenerate_dots": False,
        "task_type": "trajectory",
        "commands": ["move the red cylinder to point 2, then return to its original position"],
        "mode": "override"
    }
    # Add more experiment configs here
]

def run_experiment(exp_config):
    print(f"\n🚀 Starting experiment: {exp_config['name']}")
    payload = {
        "regenerate_scene": exp_config["regenerate_scene"],
        "regenerate_dots": exp_config["regenerate_dots"],
        "task_type": exp_config["task_type"],
        "commands": exp_config["commands"],
        "mode": exp_config["mode"]
    }

    # Run full experiment pipeline
    res = requests.post(f"{BASE_URL}/run_full_experiment", json=payload)
    if res.status_code != 200:
        print(f"❌ Error: {res.text}")
    else:
        print(f"✅ Finished: {exp_config['name']}")

    time.sleep(1)  # optional cooldown between runs

if __name__ == "__main__":
    for exp in EXPERIMENTS:
        run_experiment(exp)
