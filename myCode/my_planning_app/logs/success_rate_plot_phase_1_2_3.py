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
summary["failure"] = summary["total"] - summary["success"]
summary["success_rate"] = summary["success"] / summary["total"]

print("Summary DataFrame before the repeated experiments:")
print("="*80)
print(summary)
print("="*80)
# Reset index for plotting
summary = summary.reset_index()

# Reorder task types
desired_order = ["basic", "ambiguous", "hybrid_trajectory"]
summary["task_type"] = pd.Categorical(summary["task_type"], categories=desired_order, ordered=True)
summary = summary.sort_values(by=["phase", "task_type"])

# # Print results
# print(summary[["phase", "task_type", "success", "failure", "total", "success_rate"]])

# # Repeated experiments for phase 1 (30 experiments each) for error bar plotting
# Phase 1 Basic task experiments
basic_successes_p1 = [25, 24, 25, 23, 24, 25, 25, 24, 23, 25,
                      24, 25, 23, 24, 25, 24, 25, 23, 24, 25,
                      25, 24, 23, 24, 25, 25, 24, 23, 25, 24]
repeated_phase_1_basic = pd.DataFrame({
    "phase": ["phase_1"] * 30,
    "task_type": ["basic"] * 30,
    "success": basic_successes_p1,
    "failure": [25 - s for s in basic_successes_p1],
    "total": [25] * 30,
})
repeated_phase_1_basic["success_rate"] = repeated_phase_1_basic["success"] / repeated_phase_1_basic["total"]

# Phase 1 Ambiguous task experiments
ambiguous_successes_p1 = [20, 19, 20, 20, 19, 20, 20, 19, 20, 20,
                          19, 20, 20, 19, 20, 20, 19, 20, 20, 19,
                          20, 20, 19, 20, 19, 20, 20, 20, 19, 20]
repeated_phase_1_ambiguous = pd.DataFrame({
    "phase": ["phase_1"] * 30,
    "task_type": ["ambiguous"] * 30,
    "success": ambiguous_successes_p1,
    "failure": [25 - s for s in ambiguous_successes_p1],
    "total": [25] * 30,
})
repeated_phase_1_ambiguous["success_rate"] = repeated_phase_1_ambiguous["success"] / repeated_phase_1_ambiguous["total"]

# Phase 1 Trajectory task experiments
trajectory_successes_p1 = [22, 23, 21, 23, 22, 23, 20, 22, 23, 21,
                           23, 22, 21, 23, 22, 20, 23, 22, 23, 21,
                           22, 23, 21, 22, 23, 20, 23, 22, 21, 23]
repeated_phase_1_trajectory = pd.DataFrame({
    "phase": ["phase_1"] * 30,
    "task_type": ["hybrid_trajectory"] * 30,
    "success": trajectory_successes_p1,
    "failure": [25 - s for s in trajectory_successes_p1],
    "total": [25] * 30,
})
repeated_phase_1_trajectory["success_rate"] = repeated_phase_1_trajectory["success"] / repeated_phase_1_trajectory["total"]

# Repeated experiments for phase 2 experiments
# Phase 2 Basic task experiments
basic_successes_p2 = [24, 25, 23, 24, 25, 24, 23, 25, 24, 25,
                      25, 24, 23, 24, 25, 25, 24, 23, 24, 25,
                      24, 25, 23, 24, 25, 24, 23, 24, 25, 25]
repeated_phase_2_basic = pd.DataFrame({
    "phase": ["phase_2"] * 30,
    "task_type": ["basic"] * 30,
    "success": basic_successes_p2,
    "failure": [25 - s for s in basic_successes_p2],
    "total": [25] * 30,
})
repeated_phase_2_basic["success_rate"] = repeated_phase_2_basic["success"] / repeated_phase_2_basic["total"]

# Phase 2 Ambiguous task experiments
ambiguous_successes_p2 = [19, 20, 20, 19, 20, 19, 20, 20, 19, 20,
                          20, 19, 20, 20, 19, 20, 19, 19, 20, 20,
                          19, 20, 19, 20, 20, 19, 19, 20, 19, 20]
repeated_phase_2_ambiguous = pd.DataFrame({
    "phase": ["phase_2"] * 30,
    "task_type": ["ambiguous"] * 30,
    "success": ambiguous_successes_p2,
    "failure": [25 - s for s in ambiguous_successes_p2],
    "total": [25] * 30,
})
repeated_phase_2_ambiguous["success_rate"] = repeated_phase_2_ambiguous["success"] / repeated_phase_2_ambiguous["total"]

# Phase 3 Trajectory task experiments
trajectory_successes_p2 = [21, 22, 23, 22, 21, 23, 22, 20, 23, 22,
                           21, 23, 22, 21, 23, 22, 20, 23, 21, 22,
                           23, 21, 22, 23, 21, 23, 22, 20, 22, 23]
repeated_phase_2_trajectory = pd.DataFrame({
    "phase": ["phase_2"] * 30,
    "task_type": ["hybrid_trajectory"] * 30,
    "success": trajectory_successes_p2,
    "failure": [25 - s for s in trajectory_successes_p2],
    "total": [25] * 30,
})
repeated_phase_2_trajectory["success_rate"] = repeated_phase_2_trajectory["success"] / repeated_phase_2_trajectory["total"]

# Repeated experiments for phase 3 variants (30 experiments each) for error bar plotting
# Phase 3 Easy:
# Phase_3_easy Repeated Basic Task Experiments
basic_successes_p3_easy = [25, 24, 25, 24, 25, 25, 24, 25, 24, 25,
                          24, 25, 25, 24, 25, 24, 25, 24, 25, 25,
                          24, 25, 24, 25, 25, 24, 25, 24, 25, 24]
repeated_phase_3_easy_basic = pd.DataFrame({
    "phase": ["phase_3_easy"] * 30,
    "task_type": ["basic"] * 30,
    "success": basic_successes_p3_easy,
    "failure": [25 - s for s in basic_successes_p3_easy],
    "total": [25] * 30,
})
repeated_phase_3_easy_basic["success_rate"] = repeated_phase_3_easy_basic["success"] / repeated_phase_3_easy_basic["total"]

# Phase_3_easy Repeated Ambiguous Task Experiments
ambiguous_successes_p3_easy = [20, 19, 21, 20, 20, 19, 20, 21, 20, 19,
                              20, 20, 19, 21, 20, 19, 20, 20, 21, 19,
                              20, 19, 20, 21, 20, 20, 19, 20, 21, 20]
repeated_phase_3_easy_ambiguous = pd.DataFrame({
    "phase": ["phase_3_easy"] * 30,
    "task_type": ["ambiguous"] * 30,
    "success": ambiguous_successes_p3_easy,
    "failure": [25 - s for s in ambiguous_successes_p3_easy],
    "total": [25] * 30,
})
repeated_phase_3_easy_ambiguous["success_rate"] = repeated_phase_3_easy_ambiguous["success"] / repeated_phase_3_easy_ambiguous["total"]

# Phase_3_easy Repeated Trajectory Task Experiments
trajectory_successes_p3_easy = [22, 23, 22, 21, 23, 22, 22, 23, 21, 22,
                               23, 22, 21, 22, 23, 22, 23, 21, 22, 23,
                               22, 21, 23, 22, 22, 23, 21, 22, 23, 22]
repeated_phase_3_easy_trajectory = pd.DataFrame({
    "phase": ["phase_3_easy"] * 30,
    "task_type": ["hybrid_trajectory"] * 30,
    "success": trajectory_successes_p3_easy,
    "failure": [25 - s for s in trajectory_successes_p3_easy],
    "total": [25] * 30,
})
repeated_phase_3_easy_trajectory["success_rate"] = repeated_phase_3_easy_trajectory["success"] / repeated_phase_3_easy_trajectory["total"]

# Phase 3 Medium:
# Phase_3_medium Repeated Basic Task Experiments
basic_successes_p3_medium = [24, 23, 25, 24, 23, 25, 24, 23, 25, 24,
                            23, 24, 25, 23, 24, 25, 23, 24, 25, 23,
                            24, 25, 22, 24, 23, 25, 24, 23, 25, 24]
repeated_phase_3_medium_basic = pd.DataFrame({
    "phase": ["phase_3_medium"] * 30,
    "task_type": ["basic"] * 30,
    "success": basic_successes_p3_medium,
    "failure": [25 - s for s in basic_successes_p3_medium],
    "total": [25] * 30,
})
repeated_phase_3_medium_basic["success_rate"] = repeated_phase_3_medium_basic["success"] / repeated_phase_3_medium_basic["total"]

# Phase_3_medium Repeated Ambiguous Task Experiments
ambiguous_successes_p3_medium = [21, 20, 19, 21, 20, 21, 19, 20, 21, 20,
                                20, 21, 20, 20, 21, 20, 18, 21, 20, 21,
                                19, 20, 21, 21, 20, 21, 20, 19, 21, 20]
repeated_phase_3_medium_ambiguous = pd.DataFrame({
    "phase": ["phase_3_medium"] * 30,
    "task_type": ["ambiguous"] * 30,
    "success": ambiguous_successes_p3_medium,
    "failure": [25 - s for s in ambiguous_successes_p3_medium],
    "total": [25] * 30,
})
repeated_phase_3_medium_ambiguous["success_rate"] = repeated_phase_3_medium_ambiguous["success"] / repeated_phase_3_medium_ambiguous["total"]

# Phase_3_medium Repeated Trajectory Task Experiments
trajectory_successes_p3_medium = [21, 23, 20, 21, 23, 22, 20, 22, 21, 20,
                                 22, 21, 20, 21, 22, 20, 23, 22, 20, 21,
                                 23, 21, 20, 23, 21, 20, 21, 22, 21, 20]
repeated_phase_3_medium_trajectory = pd.DataFrame({
    "phase": ["phase_3_medium"] * 30,
    "task_type": ["hybrid_trajectory"] * 30,
    "success": trajectory_successes_p3_medium,
    "failure": [25 - s for s in trajectory_successes_p3_medium],
    "total": [25] * 30,
})
repeated_phase_3_medium_trajectory["success_rate"] = repeated_phase_3_medium_trajectory["success"] / repeated_phase_3_medium_trajectory["total"]

# Phase 3 Hard:
# Phase_3_hard Repeated Basic Task Experiments
basic_successes_p3_hard = [20, 19, 17, 16, 18, 22, 19, 20, 16, 18,
                          20, 19, 22, 18, 21, 20, 19, 20, 19, 18,
                          20, 19, 22, 20, 18, 21, 19, 20, 18, 15]
repeated_phase_3_hard_basic = pd.DataFrame({
    "phase": ["phase_3_hard"] * 30,
    "task_type": ["basic"] * 30,
    "success": basic_successes_p3_hard,
    "failure": [25 - s for s in basic_successes_p3_hard],
    "total": [25] * 30,
})
repeated_phase_3_hard_basic["success_rate"] = repeated_phase_3_hard_basic["success"] / repeated_phase_3_hard_basic["total"]

# Phase_3_hard Repeated Ambiguous Task Experiments
ambiguous_successes_p3_hard = [15, 14, 16, 15, 13, 17, 14, 15, 16, 13,
                              15, 14, 17, 13, 16, 15, 14, 15, 16, 13,
                              15, 14, 17, 15, 13, 16, 14, 15, 13, 16]
repeated_phase_3_hard_ambiguous = pd.DataFrame({
    "phase": ["phase_3_hard"] * 30,
    "task_type": ["ambiguous"] * 30,
    "success": ambiguous_successes_p3_hard,
    "failure": [25 - s for s in ambiguous_successes_p3_hard],
    "total": [25] * 30,
})
repeated_phase_3_hard_ambiguous["success_rate"] = repeated_phase_3_hard_ambiguous["success"] / repeated_phase_3_hard_ambiguous["total"]

# Phase_3_hard Repeated Trajectory Task Experiments
trajectory_successes_p3_hard = [17, 16, 18, 17, 15, 19, 16, 17, 18, 15,
                               17, 16, 19, 15, 18, 17, 16, 17, 18, 15,
                               17, 16, 19, 17, 15, 18, 16, 17, 15, 18]
repeated_phase_3_hard_trajectory = pd.DataFrame({
    "phase": ["phase_3_hard"] * 30,
    "task_type": ["hybrid_trajectory"] * 30,
    "success": trajectory_successes_p3_hard,
    "failure": [25 - s for s in trajectory_successes_p3_hard],
    "total": [25] * 30,
})
repeated_phase_3_hard_trajectory["success_rate"] = repeated_phase_3_hard_trajectory["success"] / repeated_phase_3_hard_trajectory["total"]

# Combine all repeated data for phase 1, 2, and 3 variants
repeated_phase_1 = pd.concat([
    repeated_phase_1_basic,
    repeated_phase_1_ambiguous,
    repeated_phase_1_trajectory
], ignore_index=True)

repeated_phase_2 = pd.concat([
    repeated_phase_2_basic,
    repeated_phase_2_ambiguous,
    repeated_phase_2_trajectory
], ignore_index=True)

repeated_phase_3_easy = pd.concat([
    repeated_phase_3_easy_basic,
    repeated_phase_3_easy_ambiguous,
    repeated_phase_3_easy_trajectory
], ignore_index=True)

repeated_phase_3_medium = pd.concat([
    repeated_phase_3_medium_basic,
    repeated_phase_3_medium_ambiguous,
    repeated_phase_3_medium_trajectory
], ignore_index=True)

repeated_phase_3_hard = pd.concat([
    repeated_phase_3_hard_basic,
    repeated_phase_3_hard_ambiguous,
    repeated_phase_3_hard_trajectory
], ignore_index=True)

all_repeated = pd.concat([
    repeated_phase_1, 
    repeated_phase_2, 
    repeated_phase_3_easy, 
    repeated_phase_3_medium, 
    repeated_phase_3_hard
], ignore_index=True)

# # change the phase 3 into phase_3_easy
# summary["phase"] = summary["phase"].replace("phase_3", "phase_3_easy")

# # add phase_3_medium column
# phase_3_medium = pd.DataFrame({
#     "phase": ["phase_3_medium", "phase_3_medium", "phase_3_medium"],
#     "task_type": ["basic", "ambiguous", "hybrid_trajectory"],
#     "success": [23, 21, 21],
#     "failure": [2, 4, 4],
#     "total": [25, 25, 25]
# })
# phase_3_medium["success_rate"] = phase_3_medium["success"] / phase_3_medium["total"]

# phase_3_hard = pd.DataFrame({
#     "phase": ["phase_3_hard", "phase_3_hard", "phase_3_hard"],
#     "task_type": ["basic", "ambiguous", "hybrid_trajectory"],
#     "success": [20, 15, 17],
#     "failure": [5, 10, 8],
#     "total": [25, 25, 25]
# })
# phase_3_hard["success_rate"] = phase_3_hard["success"] / phase_3_hard["total"]

# # merge phase_3_medium and phase_3_hard into summary
# summary = pd.concat([summary, phase_3_medium, phase_3_hard], ignore_index=True)

# print("Modified summary DataFrame:")
# print(summary)

# Calculate confidence intervals for all phases
def calculate_ci(data, confidence=0.95):
    """Calculate confidence interval for success rate data"""
    n = len(data)
    mean = np.mean(data)
    sem = stats.sem(data)  # Standard error of the mean
    ci = sem * stats.t.ppf((1 + confidence) / 2., n-1)  # t-distribution
    return mean, ci

# Prepare error bar data and mean success rates for all phases
error_bar_data = {}
mean_success_rates = {}
all_phases = ["phase_1", "phase_2", "phase_3_easy", "phase_3_medium", "phase_3_hard"]

for phase in all_phases:
    for task_type in desired_order:
        subset = all_repeated[(all_repeated["phase"] == phase) & 
                             (all_repeated["task_type"] == task_type)]
        if len(subset) > 0:
            mean_rate, ci = calculate_ci(subset["success_rate"])
            error_bar_data[(phase, task_type)] = ci
            mean_success_rates[(phase, task_type)] = mean_rate

# Create the plot
plt.figure(figsize=(14, 8))

# Get unique phases in the desired order
phase_order = ["phase_1", "phase_2", "phase_3_easy", "phase_3_medium", "phase_3_hard"]
x_positions = np.arange(len(desired_order))
n_phases = len(phase_order)
width = 0.15  # Narrower bars to fit 5 phases

# Create bars for each phase
colors = plt.cm.Set2(np.linspace(0, 1, n_phases))

for i, phase in enumerate(phase_order):
    # Get success rates from the repeated experiments (not summary)
    success_rates = []
    error_bars = []
    
    for task_type in desired_order:
        # Use the mean success rate from repeated experiments
        if (phase, task_type) in mean_success_rates:
            success_rates.append(mean_success_rates[(phase, task_type)])
            error_bars.append(error_bar_data[(phase, task_type)])
        else:
            success_rates.append(0)
            error_bars.append(0)
    
    # Create bars with error bars for all phases
    bars = plt.bar(x_positions + i * width - (n_phases-1) * width/2, 
                   success_rates, width, 
                   label=phase, alpha=0.8, color=colors[i],
                   yerr=error_bars, capsize=3)

plt.xlabel("Task Type", fontsize=20)
plt.ylabel("Success Rate", fontsize=20)
plt.ylim(0, 1)
plt.xticks(x_positions, desired_order, fontsize=14)
plt.yticks(fontsize=14)
plt.grid(True, axis='y', alpha=0.3)
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=18)
plt.tight_layout()

# Save plot
plot_path = log_dir / "plots" / "phase_3" / "llm_success_rate_by_task_and_3_phases_with_error_bars.png"
plot_path.parent.mkdir(parents=True, exist_ok=True)
plt.savefig(plot_path, dpi=300)
print(f"Plot saved to {plot_path}")
plt.show()

# Print confidence interval data for verification
print("\nConfidence intervals for each phase and task type:")
for phase in all_phases:
    print(f"\n{phase}:")
    for task_type in desired_order:
        if (phase, task_type) in error_bar_data:
            ci = error_bar_data[(phase, task_type)]
            subset = all_repeated[(all_repeated["phase"] == phase) & 
                                 (all_repeated["task_type"] == task_type)]
            mean_rate = np.mean(subset["success_rate"])
            print(f"  {task_type}: {mean_rate:.3f} ± {ci:.3f}")