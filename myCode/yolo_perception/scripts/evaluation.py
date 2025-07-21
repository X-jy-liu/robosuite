from pathlib import Path
import numpy as np
from tqdm import tqdm
from collections import defaultdict
import matplotlib.pyplot as plt
import os
from scipy.optimize import linear_sum_assignment

ref_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/labels")
pred_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/inference_from_mini_train/labels")
plots_save_path = Path("/home/s2644572/robosuite/myCode/yolo_perception/plots/")
os.makedirs(plots_save_path, exist_ok=True)

threshold = 0.00125  # meters
table_size_m = 0.8  # meters
norm_threshold = threshold / table_size_m  # normalize threshold
print("threshold in meters:", threshold)
print(f"Normalized threshold: {norm_threshold:.6f} over table size {table_size_m}m")

num_classes = 6

# Stats
total_gt_per_class = np.zeros(num_classes, dtype=int)
correct_class_pred_per_class = np.zeros(num_classes, dtype=int)
correct_position_pred_per_class = np.zeros(num_classes, dtype=int)
position_errors_per_class = defaultdict(list)  # class_id → list of distances
mis_class_file_lst = defaultdict(list)  # class_id → list of misclassified files

class_names = [
    "red_cube",
    "green_cube",
    "blue_cube",
    "red_cylinder",
    "green_cylinder",
    "blue_cylinder",
]

if_break = False  # Set to True to stop on first misclassification

for ref_file in tqdm(ref_dir.glob("*.txt"), desc="Evaluating predictions"):
    pred_file = pred_dir / ref_file.name
    if not pred_file.exists():
        print(f"Missing prediction: {pred_file.name}")
        continue

    with open(ref_file) as f1, open(pred_file) as f2:
        ref_lines = [line.strip().split() for line in f1.readlines()]
        pred_lines = [line.strip().split() for line in f2.readlines()]

    for r_line, p_line in zip(ref_lines, pred_lines):
        class_r, x_r, y_r = int(r_line[0]), float(r_line[1]), float(r_line[2])
        class_p, x_p, y_p = int(p_line[0]), float(p_line[1]), float(p_line[2])

        # Update stats
        total_gt_per_class[class_r] += 1

        if class_r == class_p:
            correct_class_pred_per_class[class_r] += 1
            dist = np.sqrt((x_r - x_p)**2 + (y_r - y_p)**2)
            correct_position_pred_per_class[class_r] += dist <= norm_threshold
            position_errors_per_class[class_r].append(dist)
        else:
            print(f"Misclassification: {ref_file.name} | GT: {class_names[class_r]} | Pred: {class_names[class_p]}")
            if_break = True
        if if_break:
            break
    if if_break:
        break
# ==== Final Report ====
print("\n📊 Class-wise Evaluation Report:")
for i in range(num_classes):
    name = class_names[i]
    total = total_gt_per_class[i]
    class_correct = correct_class_pred_per_class[i]
    pos_correct = correct_position_pred_per_class[i]

    class_acc = class_correct / total if total > 0 else 0
    pos_acc = pos_correct / class_correct if class_correct > 0 else 0

    print(f"Class: {name}")
    print(f"  Classification Accuracy: {class_acc:.2%} ({class_correct}/{total})")
    print(f"  Position Accuracy (within {threshold}m): {pos_acc:.2%} ({pos_correct}/{class_correct})\n")

# ==== plot position errors ====

print("\n📈 Plotting Position Error Distributions:")
use_density = True  # Set to False if you want raw counts
for i in range(num_classes):
    errors = position_errors_per_class[i]
    if not errors:
        print(f"Skipping empty class: {class_names[i]}")
        continue

    plt.figure(figsize=(6, 4))
    plt.hist(
        errors,
        bins=50,
        alpha=0.75,
        density=use_density,
        color="steelblue",
        edgecolor="black"
    )

    ylabel = "Density" if use_density else "Frequency"
    plt.xlabel("Euclidean Distance Error (m)")
    plt.ylabel(ylabel)
    plt.title(f"Position Error Distribution: {class_names[i]}")
    plt.grid(True)

    # Save plot
    filename = plots_save_path / f"position_error_{class_names[i]}.png"
    plt.tight_layout()
    plt.savefig(filename)
    plt.close()

    print(f"✅ Saved: {filename}")