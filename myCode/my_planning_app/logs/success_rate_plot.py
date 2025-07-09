from pathlib import Path
import json
import pandas as pd

# Setup
log_dir = Path("/home/jingyang/robosuite/myCode/my_planning_app/logs")
valid_prefixes = ("ambiguous", "basic", "trajectory")
target_suffix = "evaluated"
results = []

# Recursively find relevant evaluated JSONs
json_paths = [
    p for p in log_dir.rglob("*.json")
    if p.name.startswith(valid_prefixes) and p.stem.endswith(target_suffix)
]
for path in json_paths:
    if path.name.startswith('basic_') and path.stem.endswith('evaluated'):
        print(f"Processing {path}...")

# Extract task_type and success_status
for path in json_paths:
    try:
        with open(path, "r") as f:
            data = json.load(f)
            task_type = data.get("task_type", "unknown")
            status = data.get("success_status", "unknown")
            if task_type in valid_prefixes and status in ("success", "failure"):
                results.append({"task_type": task_type, "status": status})
    except Exception as e:
        print(f"Failed to process {path.name}: {e}")

# Convert to DataFrame
df = pd.DataFrame(results)

# Compute success rate
summary = df.groupby("task_type")["status"].value_counts().unstack().fillna(0)
summary["total"] = summary.sum(axis=1)
summary["success_rate"] = summary["success"] / summary["total"]

# Print results
print(summary[["success", "failure", "total", "success_rate"]].sort_index())

# Optionally: Plot
import matplotlib.pyplot as plt

summary["success_rate"].plot(kind="bar", color="skyblue", figsize=(8, 5))
plt.title("Success Rate by Task Type")
plt.ylabel("Success Rate")
plt.ylim(0, 1)
plt.xticks(rotation=0)
plt.grid(True, axis='y')
plt.tight_layout()

# Save plot
plot_path = log_dir / "llm_success_rate_by_task.png"
plt.savefig(plot_path, dpi=300)
plt.show()
