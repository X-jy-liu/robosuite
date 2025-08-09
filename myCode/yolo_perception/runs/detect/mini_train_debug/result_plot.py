import pandas as pd
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv("results.csv")

# Setup plot style
plt.style.use('ggplot')  # or 'bmh', 'classic', 'default'
plt.figure(figsize=(8, 5))

# Plot precision with smoother visuals
plt.plot(df["epoch"], df["metrics/precision(B)"], color='royalblue', linewidth=2, marker='o', markersize=4, label="Precision (B)")

# Formatting
# plt.title("Precision Convergence on Mini Dataset", fontsize=14, weight='bold')
plt.xlabel("Epoch", fontsize=18)
plt.ylabel("Precision", fontsize=18)
plt.ylim(0.0, 1.05)
plt.xticks(fontsize=10)
plt.yticks(fontsize=10)
plt.legend(loc="lower right", fontsize=10)
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()

# Save high-res version for Overleaf
plt.savefig("/home/s2644572/robosuite/myCode/yolo_perception/plots_2/precision_convergence_mini.pdf", dpi=300)
plt.show()
