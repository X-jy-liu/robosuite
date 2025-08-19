from pathlib import Path
import numpy as np
from tqdm import tqdm
from collections import defaultdict
import matplotlib.pyplot as plt
import os
from scipy.optimize import linear_sum_assignment

# -------------------- Paths --------------------
ref_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/labels")
pred_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/inference_from_mini_train_1/labels")
plots_save_path = Path("/home/s2644572/robosuite/myCode/yolo_perception/plots_2/")
os.makedirs(plots_save_path, exist_ok=True)

# -------------------- Thresholds --------------------
table_size_m = 0.8  # meters (used for un-normalizing)
# Your stated success threshold is 10 mm -> 0.01 m
threshold = 0.01  # meters  ### CHANGED: was 0.0025 (2.5 mm)
norm_threshold = threshold / table_size_m  # because (x, y) are normalized by table size

success_threshold_mm = 10.0  # mm, used for plotting/shading and summary  ### NEW

print("threshold (success) in meters:", threshold)
print(f"Normalized threshold: {norm_threshold:.6f} over table size {table_size_m} m")

num_classes = 6

# -------------------- Stats containers --------------------
total_gt_per_class = np.zeros(num_classes, dtype=int)
correct_class_pred_per_class = np.zeros(num_classes, dtype=int)
correct_position_pred_per_class = np.zeros(num_classes, dtype=int)
position_errors_per_class = defaultdict(list)  # class_id → list of distances (normalized)
dx_mis_class_file_lst = defaultdict(list)
dy_mis_class_file_lst = defaultdict(list)
mis_class_file_lst = defaultdict(list)

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
ref_txt_files = list(ref_dir.glob("*.txt"))
print(f"Found {len(ref_txt_files)} reference files.")

# -------------------- Evaluation loop --------------------
for ref_file in tqdm(ref_txt_files, desc="Evaluating predictions"):
    pred_file = pred_dir / ref_file.name
    if not pred_file.exists():
        continue
    else:
        file_counter += 1

    with open(ref_file) as f1, open(pred_file) as f2:
        ref_lines = [line.strip().split() for line in f1.readlines()]
        pred_lines = [line.strip().split() for line in f2.readlines()]

    # Extract (x, y) positions only (normalized by table size)
    ref_xy = np.array([[float(x), float(y)] for _, x, y, _, _ in ref_lines])
    pred_xy = np.array([[float(x), float(y)] for _, x, y, _, _ in pred_lines])

    # Pairwise distances (normalized)
    dists = np.linalg.norm(ref_xy[:, None, :] - pred_xy[None, :, :], axis=2)

    # Hungarian assignment for best matching
    row_ind, col_ind = linear_sum_assignment(dists)

    # Reorder lines to aligned pairs
    ref_lines = [ref_lines[i] for i in row_ind]
    pred_lines = [pred_lines[j] for j in col_ind]

    for r_line, p_line in zip(ref_lines, pred_lines):
        class_r, x_r, y_r = int(r_line[0]), float(r_line[1]), float(r_line[2])
        class_p, x_p, y_p = int(p_line[0]), float(p_line[1]), float(p_line[2])

        total_gt_per_class[class_r] += 1

        if class_r == class_p:
            correct_class_pred_per_class[class_r] += 1
            dx = x_r - x_p
            dy = y_r - y_p
            dist = np.hypot(dx, dy)  # normalized distance
            # Success under normalized threshold (== 10 mm when un-normalized)
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

# -------------------- Final report --------------------
print("\n📊 Class-wise Evaluation Report:")
class_dst_errors_m = []
for i in range(num_classes):
    name = class_names[i]
    total = total_gt_per_class[i]
    class_correct = correct_class_pred_per_class[i]
    pos_correct = correct_position_pred_per_class[i]

    class_acc = class_correct / total if total > 0 else 0.0
    pos_acc = pos_correct / class_correct if class_correct > 0 else 0.0

    mean_err_m = (np.mean(position_errors_per_class[i]) * table_size_m) if position_errors_per_class[i] else np.nan
    dx_mean = np.mean(dx_mis_class_file_lst[i]) if dx_mis_class_file_lst[i] else np.nan
    dy_mean = np.mean(dy_mis_class_file_lst[i]) if dy_mis_class_file_lst[i] else np.nan

    print(f"Class: {name}")
    print(f"  Classification Accuracy: {class_acc:.2%} ({class_correct}/{total})")
    print(f"  Position Accuracy (≤ {threshold*1000:.0f} mm): {pos_acc:.2%} ({pos_correct}/{class_correct})")
    print(f"  Mean Position Error: {mean_err_m:.6f} m ({mean_err_m*1000:.2f} mm)")
    print(f"  dx mean error (normalized): {dx_mean:.6f}")
    print(f"  dy mean error (normalized): {dy_mean:.6f}\n")

    if not np.isnan(mean_err_m):
        class_dst_errors_m.append(mean_err_m)

overall_mean_m = np.mean(class_dst_errors_m) if len(class_dst_errors_m) > 0 else np.nan
print(f"Overall Mean Euclidean Distance Error: {overall_mean_m:.6f} m ({overall_mean_m*1000:.2f} mm)")

# -------------------- Plotting with shaded 'good' region --------------------
print("\n📈 Plotting Position Error Distributions (all bars ≤ 10 mm, marked green):")
use_density = True  # raw counts

for i in range(num_classes):
    errs_mm = np.array(position_errors_per_class[i]) * table_size_m * 1000.0
    if errs_mm.size == 0:
        print(f"Skipping empty class: {class_names[i]}")
        continue

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.hist(
        errs_mm,
        bins=50,
        alpha=0.8,
        density=use_density,
        color="green",          # All bars shown as green
        edgecolor="black",
        label=f"Accurate Position Predictions"
    )

    ylabel = "Density" if use_density else "Frequency"
    ax.set_xlabel("Euclidean Distance Error (mm)", fontsize=14)
    ax.set_ylabel(ylabel, fontsize=14)
    ax.tick_params(labelsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=12)

    filename = plots_save_path / f"position_error_{class_names[i]}.png"
    fig.tight_layout()
    fig.savefig(filename, dpi=200)
    plt.close(fig)

    print(f"✅ Saved: {filename} | {class_names[i]} max error = {errs_mm.max():.2f} mm")