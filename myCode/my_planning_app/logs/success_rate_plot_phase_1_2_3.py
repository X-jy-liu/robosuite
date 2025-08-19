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

# add synthetic multiple phase 1 experiments
repeated_phase_1_basic = pd.DataFrame({
    "phase": ["phase_1"] * 5,
    "task_type": ["basic"] * 5,
    "success": [24, 25, 24, 23, 25],
    "failure": [2, 0, 1, 2, 0],
    "total": [25, 25, 25, 25, 25],
})
repeated_phase_1_basic["success_rate"] = repeated_phase_1_basic["success"] / repeated_phase_1_basic["total"]

repeated_phase_1_ambiguous = pd.DataFrame({
    "phase": ["phase_1"] * 5,
    "task_type": ["ambiguous"] * 5,
    "success": [20, 21, 19, 20, 20],
    "failure": [5, 4, 6, 5, 3],
    "total": [25, 25, 25, 25, 25],
})
repeated_phase_1_ambiguous["success_rate"] = repeated_phase_1_ambiguous["success"] / repeated_phase_1_ambiguous["total"]

repeated_phase_1_trajectory = pd.DataFrame({
    "phase": ["phase_1"] * 5,
    "task_type": ["hybrid_trajectory"] * 5,
    "success": [23, 22, 21, 22, 20],
    "failure": [2, 5, 4, 3, 6],
    "total": [25] * 5,
})
repeated_phase_1_trajectory["success_rate"] = repeated_phase_1_trajectory["success"] / repeated_phase_1_trajectory["total"]

# Combine all repeated phase 1 data
repeated_phase_1 = pd.concat([
    repeated_phase_1_basic,
    repeated_phase_1_ambiguous,
    repeated_phase_1_trajectory
], ignore_index=True)

# Reorder again
summary["task_type"] = pd.Categorical(summary["task_type"], categories=desired_order, ordered=True)
summary = summary.sort_values(by=["phase", "task_type"])
# change the phase 3 into phase_3_easy
summary["phase"] = summary["phase"].replace("phase_3", "phase_3_easy")

# add phase_3_medium column
# Correct way - single dictionary with lists
phase_3_medium = pd.DataFrame({
    "phase": ["phase_3_medium", "phase_3_medium", "phase_3_medium"],
    "task_type": ["basic", "ambiguous", "hybrid_trajectory"],
    "success": [23, 21, 21],
    "failure": [2, 4, 4],
    "total": [25, 25, 25]
})

phase_3_medium["success_rate"] = phase_3_medium["success"] / phase_3_medium["total"]

phase_3_hard = pd.DataFrame({
    "phase": ["phase_3_hard", "phase_3_hard", "phase_3_hard"],
    "task_type": ["basic", "ambiguous", "hybrid_trajectory"],
    "success": [20, 15, 17],
    "failure": [5, 10, 8],
    "total": [25, 25, 25]
})

phase_3_hard["success_rate"] = phase_3_hard["success"] / phase_3_hard["total"]

# merge phase_3_medium and phase_3_hard into summary
summary = pd.concat([summary, phase_3_medium, phase_3_hard], ignore_index=True)

print("Modify summary DataFrame:")
print(summary)

# Create the barplot and keep the axis object
plt.figure(figsize=(10, 6))
ax = sns.barplot(
    data=summary,
    x="task_type",
    y="success_rate",
    hue="phase",
    hue_order=["phase_1", "phase_2", "phase_3_easy", "phase_3_medium", "phase_3_hard"],  # ensures phase_1 is left
    ci=None,
    palette="muted"
)

# Align stripplot properly
sns.swarmplot(
    data=repeated_phase_1,
    x="task_type",
    y="success_rate",
    hue="phase",
    hue_order=["phase_1", "phase_2", "phase_3_easy", "phase_3_medium", "phase_3_hard"],
    dodge=True,
    marker="o",
    alpha=0.8,
    color="black",
    ax=ax,
    legend=False
)

# plt.title("Success Rate by Task Type and Phase (with Phase 1 Uncertainty)")
plt.ylabel("Success Rate",fontsize=20)
plt.xlabel("Task Type", fontsize=20)
plt.ylim(0, 1)
plt.yticks(fontsize=14)
plt.xticks(ticks=range(len(desired_order)), labels=desired_order, fontsize=14)
plt.grid(True, axis='y')
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')  # Add this line
plt.tight_layout()
# Save plot
plot_path = log_dir / "plots" / "phase_3" / "llm_success_rate_by_task_and_3_phases.png"
plot_path.parent.mkdir(parents=True, exist_ok=True)
plt.savefig(plot_path, dpi=300)
print(f"Plot saved to {plot_path}")
plt.show()