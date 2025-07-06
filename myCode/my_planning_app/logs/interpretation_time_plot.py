from pathlib import Path
import json
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

# Directory setup
experiment_dir = Path("/home/jingyang/robosuite/myCode/my_planning_app/logs")
valid_prefixes = ("ambiguous", "basic", "trajectory")
data = []

# Filter files: start with valid prefix, end with .json, and not *_evaluated.json
json_paths = [
    p for p in experiment_dir.rglob("*.json")
    if p.name.startswith(valid_prefixes) and not p.stem.endswith("evaluated")
]

# Read and extract interpretation times
for filepath in json_paths:
    try:
        with open(filepath, "r") as f:
            content = json.load(f)
            for step in content.get("steps", []):
                time = step.get("llm_interpretation_time_sec")
                task_type = step.get("task_type", "unknown")
                if time is not None and task_type in valid_prefixes:
                    data.append({
                        "task_type": task_type,
                        "interpretation_time": time
                    })
    except Exception as e:
        print(f"Failed to process {filepath.name}: {e}")

# Convert to DataFrame
df = pd.DataFrame(data)

# Plot boxplot grouped by task_type
plt.figure(figsize=(10, 6))
sns.boxplot(data=df, 
            x="task_type", 
            y="interpretation_time",
            order=["basic", "ambiguous", "trajectory"])
plt.title("LLM Interpretation Time by Task Type")
plt.xlabel("Task Type")
plt.ylabel("Interpretation Time (sec)")
plt.grid(True)
plt.tight_layout()

# Save plot
img_path = experiment_dir / "llm_interpretation_time_boxplot_by_task.png"
plt.savefig(img_path, dpi=300)
plt.show()