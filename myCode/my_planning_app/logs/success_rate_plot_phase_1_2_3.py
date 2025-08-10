from pathlib import Path
import json
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.pyplot as plt

# Setup
log_dir = Path.cwd()
valid_prefixes = ("ambiguous", "basic", "trajectory")
target_suffix = "evaluated"
results = []

# Iterate through all scene folders
for scene_dir in log_dir.rglob("phase_*"):
    if not scene_dir.is_dir():
        continue
    phase = scene_dir.name  # phase_1 or phase_2
    for path in scene_dir.rglob("*.json"):
        if path.name.startswith(valid_prefixes) and path.stem.endswith(target_suffix):
            try:
                with open(path, "r") as f:
                    data = json.load(f)
                    task_type = data.get("task_type", "unknown")
                    status = data.get("success_status", "unknown")
                    if task_type in valid_prefixes and status in ("success", "failure"):
                        if task_type == "trajectory":
                            task_type = "hybrid_trajectory"
                        results.append({
                            "phase": phase,
                            "task_type": task_type,
                            "status": status
                        })
            except Exception as e:
                print(f"Failed to process {path.name}: {e}")

# Convert to DataFrame
df = pd.DataFrame(results)

# Compute success rates per phase and task_type
summary = df.groupby(["phase", "task_type"])["status"].value_counts().unstack().fillna(0)
summary["total"] = summary.sum(axis=1)
# set the total number of experiments for each task type as 25
summary["total"] = 25
summary["success_rate"] = summary["success"] / summary["total"]

# Reset index for plotting
summary = summary.reset_index()

# Reorder task types
desired_order = ["basic", "ambiguous", "hybrid_trajectory"]
summary["task_type"] = pd.Categorical(summary["task_type"], categories=desired_order, ordered=True)
summary = summary.sort_values(by=["phase", "task_type"])

# Print results
print(summary[["phase", "task_type", "success", "failure", "total", "success_rate"]])

# Create the barplot and keep the axis object
plt.figure(figsize=(10, 6))
ax = sns.barplot(
    data=summary,
    x="task_type",
    y="success_rate",
    hue="phase",
    hue_order=["phase_1", "phase_2", "phase_3"],  # ensures phase_1 is left
    ci=None,
    palette="muted"
)

# plt.title("Success Rate by Task Type and Phase (with Phase 1 Uncertainty)")
plt.ylabel("Success Rate",fontsize=20)
plt.xlabel("Task Type", fontsize=20)
plt.ylim(0, 1)
plt.yticks(fontsize=14)
plt.xticks(ticks=range(len(desired_order)), labels=desired_order, fontsize=14)
plt.grid(True, axis='y')
plt.tight_layout()
# Save plot
plot_path = log_dir / "plots" / "phase_3" / "llm_success_rate_by_task_and_3_phases.png"
plot_path.parent.mkdir(parents=True, exist_ok=True)
plt.savefig(plot_path, dpi=300)
print(f"Plot saved to {plot_path}")
plt.show()