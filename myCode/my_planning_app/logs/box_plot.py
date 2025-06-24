import os
import json
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

# Set the directory containing your JSON files
experiment_dir = "/home/jingyang/robosuite/myCode/my_planning_app/logs"  # <-- CHANGE THIS

# Initialize a list to collect all interpretation times and types
data = []

# Loop through files
for filename in os.listdir(experiment_dir):
    if filename.endswith(".json"):
        filepath = os.path.join(experiment_dir, filename)
        with open(filepath, "r") as f:
            try:
                content = json.load(f)
                experiment_type = filename.split("_")[0]  # e.g., 'ambiguous'
                for step in content.get("steps", []):
                    time = step.get("llm_interpretation_time_sec")
                    if time is not None:
                        data.append({
                            "experiment_type": experiment_type,
                            "interpretation_time": time
                        })
            except Exception as e:
                print(f"Failed to process {filename}: {e}")

# Convert to DataFrame
df = pd.DataFrame(data)

# Plot boxplot
plt.figure(figsize=(10, 6))
sns.boxplot(data=df, x="experiment_type", y="interpretation_time")
plt.title("LLM Interpretation Time by Experiment Type")
plt.xlabel("Experiment Type")
plt.ylabel("Interpretation Time (sec)")
plt.grid(True)
plt.tight_layout()

img_path = "/home/jingyang/robosuite/myCode/my_planning_app/logs/llm_interpretation_time_boxplot.png"
# Save the plot
plt.savefig(img_path, dpi=300)

# Optionally show it
plt.show()
