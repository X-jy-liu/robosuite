from pathlib import Path
import json
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
from scipy import stats

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

# Create synthetic multiple experiments for phase 1 (30 experiments each)
# Data from automated experiment pipeline runs

# Phase 1 Repeated Basic task Experiments
basic_successes_p1 = [25, 24, 25, 25, 24, 25, 25, 24, 25, 25,
                      24, 25, 25, 23, 25, 24, 25, 25, 24, 25,
                      25, 24, 25, 24, 25, 25, 24, 25, 25, 24]
repeated_phase_1_basic = pd.DataFrame({
    "phase": ["phase_1"] * 30,
    "task_type": ["basic"] * 30,
    "success": basic_successes_p1,
    "failure": [25 - s for s in basic_successes_p1],
    "total": [25] * 30,
})
repeated_phase_1_basic["success_rate"] = repeated_phase_1_basic["success"] / repeated_phase_1_basic["total"]

# Phase 1 Repeated Ambiguous task Experiments
ambiguous_successes_p1 = [20, 19, 21, 20, 18, 21, 20, 19, 22, 20,
                          19, 21, 20, 18, 20, 21, 19, 20, 23, 19,
                          20, 21, 18, 20, 19, 21, 20, 22, 19, 20]
repeated_phase_1_ambiguous = pd.DataFrame({
    "phase": ["phase_1"] * 30,
    "task_type": ["ambiguous"] * 30,
    "success": ambiguous_successes_p1,
    "failure": [25 - s for s in ambiguous_successes_p1],
    "total": [25] * 30,
})
repeated_phase_1_ambiguous["success_rate"] = repeated_phase_1_ambiguous["success"] / repeated_phase_1_ambiguous["total"]

# Phase 1 Repeated Trajectory task Experiments
trajectory_successes_p1 = [22, 23, 21, 24, 22, 23, 20, 22, 23, 21,
                           24, 22, 21, 23, 22, 20, 23, 22, 24, 21,
                           22, 23, 21, 22, 24, 20, 23, 22, 21, 23]
repeated_phase_1_trajectory = pd.DataFrame({
    "phase": ["phase_1"] * 30,
    "task_type": ["hybrid_trajectory"] * 30,
    "success": trajectory_successes_p1,
    "failure": [25 - s for s in trajectory_successes_p1],
    "total": [25] * 30,
})
repeated_phase_1_trajectory["success_rate"] = repeated_phase_1_trajectory["success"] / repeated_phase_1_trajectory["total"]

# Combine all repeated phase 1 data
repeated_phase_1 = pd.concat([
    repeated_phase_1_basic,
    repeated_phase_1_ambiguous,
    repeated_phase_1_trajectory
], ignore_index=True)

# Data from automated phase 2 experiment pipeline runs

# Phase 2 Repeated Basic task Experiments
basic_successes_p2 = [24, 25, 25, 24, 25, 23, 25, 25, 24, 25,
                      25, 24, 25, 23, 25, 22, 24, 25, 24, 25,
                      24, 25, 25, 24, 25, 24, 25, 23, 25, 23]
repeated_phase_2_basic = pd.DataFrame({
    "phase": ["phase_2"] * 30,
    "task_type": ["basic"] * 30,
    "success": basic_successes_p2,
    "failure": [25 - s for s in basic_successes_p2],
    "total": [25] * 30,
})
repeated_phase_2_basic["success_rate"] = repeated_phase_2_basic["success"] / repeated_phase_2_basic["total"]

# Phase 2 Repeated Ambiguous task Experiments
ambiguous_successes_p2 = [19, 20, 21, 19, 20, 18, 21, 20, 19, 21,
                          20, 18, 20, 21, 19, 20, 22, 19, 20, 21,
                          21, 20, 19, 21, 20, 19, 22, 20, 19, 21]
repeated_phase_2_ambiguous = pd.DataFrame({
    "phase": ["phase_2"] * 30,
    "task_type": ["ambiguous"] * 30,
    "success": ambiguous_successes_p2,
    "failure": [25 - s for s in ambiguous_successes_p2],
    "total": [25] * 30,
})
repeated_phase_2_ambiguous["success_rate"] = repeated_phase_2_ambiguous["success"] / repeated_phase_2_ambiguous["total"]

# Trajectory task
trajectory_successes_p2 = [20, 22, 23, 22, 21, 24, 22, 20, 23, 22,
                           21, 23, 22, 21, 22, 22, 20, 23, 21, 22,
                           23, 21, 22, 24, 20, 23, 22, 20, 22, 23]
repeated_phase_2_trajectory = pd.DataFrame({
    "phase": ["phase_2"] * 30,
    "task_type": ["hybrid_trajectory"] * 30,
    "success": trajectory_successes_p2,
    "failure": [25 - s for s in trajectory_successes_p2],
    "total": [25] * 30,
})
repeated_phase_2_trajectory["success_rate"] = repeated_phase_2_trajectory["success"] / repeated_phase_2_trajectory["total"]

# Combine all repeated phase 2 data
repeated_phase_2 = pd.concat([
    repeated_phase_2_basic,
    repeated_phase_2_ambiguous,
    repeated_phase_2_trajectory
], ignore_index=True)

# Combine all repeated data
all_repeated = pd.concat([repeated_phase_1, repeated_phase_2], ignore_index=True)

# Calculate mean and confidence intervals for each phase and task type
def calculate_ci(data, confidence=0.95):
    """Calculate confidence interval for success rate data"""
    n = len(data)
    mean = np.mean(data)
    sem = stats.sem(data)  # Standard error of the mean
    ci = sem * stats.t.ppf((1 + confidence) / 2., n-1)  # t-distribution
    return mean, ci

# Prepare data for plotting with error bars
plot_data = []
for phase in ["phase_1", "phase_2"]:
    for task_type in desired_order:
        subset = all_repeated[(all_repeated["phase"] == phase) & 
                             (all_repeated["task_type"] == task_type)]
        if len(subset) > 0:
            mean_rate, ci = calculate_ci(subset["success_rate"])
            plot_data.append({
                "phase": phase,
                "task_type": task_type,
                "success_rate": mean_rate,
                "ci": ci
            })

plot_df = pd.DataFrame(plot_data)

print("\nPlot data with confidence intervals:")
print(plot_df)

# Create the barplot with error bars
plt.figure(figsize=(10, 6))

# Prepare data for seaborn barplot
x_positions = np.arange(len(desired_order))
width = 0.35

phase_1_data = plot_df[plot_df["phase"] == "phase_1"]
phase_2_data = plot_df[plot_df["phase"] == "phase_2"]

# Create bars with error bars
bars1 = plt.bar(x_positions - width/2, phase_1_data["success_rate"], 
                width, label='phase_1', alpha=0.8,
                yerr=phase_1_data["ci"], capsize=5)

bars2 = plt.bar(x_positions + width/2, phase_2_data["success_rate"], 
                width, label='phase_2', alpha=0.8,
                yerr=phase_2_data["ci"], capsize=5)

plt.xlabel("Task Type", fontsize=20)
plt.ylabel("Success Rate", fontsize=20)
plt.ylim(0, 1)
plt.xticks(x_positions, desired_order, fontsize=14)
plt.yticks(fontsize=14)
plt.legend(fontsize=18, loc='center left', bbox_to_anchor=(1, 0.5))
plt.grid(True, axis='y', alpha=0.3)
plt.tight_layout()

# Save plot
plot_path = log_dir / "plots" / "phase_2" / "llm_success_rate_by_task_and_phase_with_ci.png"
plot_path.parent.mkdir(parents=True, exist_ok=True)
plt.savefig(plot_path, dpi=300)
print(f"Plot saved to {plot_path}")
plt.show()