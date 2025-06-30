import requests
import time
from datetime import datetime

BASE_URL = "http://localhost:8000"

EXPERIMENTS = [
    {
        "name": "Exp_1_put_together",
        "regenerate_scene": True,
        "regenerate_dots": True,
        "task_type": "trajectory",
        "commands": ["put the red cube and blue cylinder together", "keep going"],
        "mode": "override"
    },
    {
        "name": "Exp_2_separate_and_group",
        "regenerate_scene": True,
        "regenerate_dots": False,
        "task_type": "trajectory",
        "commands": ["separate all objects", "now group the green ones"],
        "mode": "override"
    },
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
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_name = f"{exp_config['name']}_{timestamp}.json"
        with open(f"logs/{log_name}", "w") as f:
            f.write(res.text)

    time.sleep(1)  # optional cooldown between runs

if __name__ == "__main__":
    for exp in EXPERIMENTS:
        run_experiment(exp)
