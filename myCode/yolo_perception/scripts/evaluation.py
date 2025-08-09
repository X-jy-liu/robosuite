from pathlib import Path
import numpy as np
from tqdm import tqdm
from collections import defaultdict
import matplotlib.pyplot as plt
import os
from scipy.optimize import linear_sum_assignment

ref_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/labels")
pred_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/inference_from_mini_train_1/labels")
plots_save_path = Path("/home/s2644572/robosuite/myCode/yolo_perception/plots_2/")
os.makedirs(plots_save_path, exist_ok=True)

threshold = 0.0025  # meters - the distance threshold for position accuracy
table_size_m = 0.8  # meters
norm_threshold = threshold / table_size_m  # normalize threshold as the pred and gt are normalized by table size
print("threshold in meters:", threshold)
print(f"Normalized threshold: {norm_threshold:.6f} over table size {table_size_m}m")

num_classes = 6

# Stats
total_gt_per_class = np.zeros(num_classes, dtype=int)
correct_class_pred_per_class = np.zeros(num_classes, dtype=int)
correct_position_pred_per_class = np.zeros(num_classes, dtype=int)
position_errors_per_class = defaultdict(list)  # class_id → list of distances
dx_mis_class_file_lst = defaultdict(list)  # class_id → list of misclassified files
dy_mis_class_file_lst = defaultdict(list)  # class_id → list of misclassified files
mis_class_file_lst = defaultdict(list)  # class_id → list of misclassified files

class_names = [
    "red_cube",
    "green_cube",
    "blue_cube",
    "red_cylinder",
    "green_cylinder",
    "blue_cylinder",
]

mismatch_files = []
file_counter = 0
ref_txt_files = list(ref_dir.glob("*.txt")) # Limit to first 500 files for testing
print(f"Found {len(ref_txt_files)} reference files.")
for ref_file in tqdm(ref_txt_files, desc="Evaluating predictions"):
    pred_file = pred_dir / ref_file.name
    if not pred_file.exists():
        # print(f"Missing prediction: {pred_file.name}")
        continue
    else:
        file_counter += 1
    with open(ref_file) as f1, open(pred_file) as f2:
        ref_lines = [line.strip().split() for line in f1.readlines()]
        pred_lines = [line.strip().split() for line in f2.readlines()]
    
    # Extract (x, y) positions only
    ref_xy = np.array([[float(x), float(y)] for _, x, y, _, _ in ref_lines])
    pred_xy = np.array([[float(x), float(y)] for _, x, y, _, _ in pred_lines])
    # Compute pairwise Euclidean distances
    dists = np.linalg.norm(ref_xy[:, None, :] - pred_xy[None, :, :], axis=2)

    # Hungarian algorithm to find best match
    row_ind, col_ind = linear_sum_assignment(dists)

    # Reorder ref_lines and pred_lines based on Hungarian matching
    ref_lines = [ref_lines[i] for i in row_ind]
    pred_lines = [pred_lines[j] for j in col_ind]
    
    for r_line, p_line in zip(ref_lines, pred_lines):
        class_r, x_r, y_r = int(r_line[0]), float(r_line[1]), float(r_line[2])
        class_p, x_p, y_p = int(p_line[0]), float(p_line[1]), float(p_line[2])

        # Update stats
        total_gt_per_class[class_r] += 1

        if class_r == class_p:
            correct_class_pred_per_class[class_r] += 1
            dist = np.sqrt((x_r - x_p)**2 + (y_r - y_p)**2)
            dx = x_r - x_p
            dy = y_r - y_p
            correct_position_pred_per_class[class_r] += dist <= norm_threshold
            position_errors_per_class[class_r].append(dist)
            dx_mis_class_file_lst[class_r].append(dx)
            dy_mis_class_file_lst[class_r].append(dy)
        else:
            mismatch_files.append({
                "file": ref_file.name,
                "gt_class": class_names[class_r],
                "pred_class": class_names[class_p]
            })

print(f"Processed {file_counter} files.")

print(f"Processed {len(mismatch_files)} mismatches out of {len(ref_txt_files)} files.")
# ==== Final Report ====
print("\n📊 Class-wise Evaluation Report:")
class_dst_errors = []
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
    print(f"  Mean Position Error: {np.mean(position_errors_per_class[i])*table_size_m:.8f}m")
    print(f"  dx mean error: {np.mean(dx_mis_class_file_lst[i]):.12f} (normalized)")
    print(f"  dy mean error: {np.mean(dy_mis_class_file_lst[i]):.12f} (normalized)\n")
    class_dst_errors.append(np.mean(position_errors_per_class[i]) * table_size_m)

print("Overall Euclidean Distance Error (m):", np.mean(class_dst_errors))
# ==== plot position errors ====

print("\n📈 Plotting Position Error Distributions:")
use_density = True  # Set to False if you want raw counts
for i in range(num_classes):
    errors = np.array(position_errors_per_class[i])*table_size_m  # Convert normalized errors back to meters
    if errors is None or len(errors) == 0:
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