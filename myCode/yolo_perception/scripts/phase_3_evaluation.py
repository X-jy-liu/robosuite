from pathlib import Path
import numpy as np
from tqdm import tqdm
from collections import defaultdict
import matplotlib.pyplot as plt
import os
from scipy.optimize import linear_sum_assignment

def main(level):
    ref_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/labels")
    pred_dir = Path(f"/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/inference_from_mini_train_3_levels_realistic_noise/{level}/labels")
    plots_save_path = Path(f"/home/s2644572/robosuite/myCode/yolo_perception/plots_6_realistic_noise_comparison/{level}")
    os.makedirs(plots_save_path, exist_ok=True)

    threshold = 0.01  # meters - the distance threshold for position accuracy because separation distance between the grippers is 0.08. the objects width is 0.05. And we leave extra 0.005 as the safety margin. so the threshold = (0.08-0.05)/2 - 0.005 = 0.01
    table_size_m = 0.8  # meters
    norm_threshold = threshold / table_size_m  # normalize threshold as the pred and gt are normalized by table size
    print(f"Evaluating level: {level}")
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

    # Define colors for each class for consistent plotting
    class_colors = [
        '#FF4444',  # red_cube - red
        '#44FF44',  # green_cube - green
        '#4444FF',  # blue_cube - blue
        '#FF8888',  # red_cylinder - light red
        '#88FF88',  # green_cylinder - light green
        '#8888FF',  # blue_cylinder - light blue
    ]

    mismatch_files = []
    file_counter = 0
    ref_txt_files = list(ref_dir.glob("*.txt"))
    print(f"Found {len(ref_txt_files)} reference files.")

    for ref_file in tqdm(ref_txt_files, desc="Evaluating predictions"):
        pred_file = pred_dir / f"{level}_{ref_file.name}"
        if not pred_file.exists():
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
        print(f"  Position Accuracy (within {threshold}m): {pos_acc:.2%} ({pos_correct}/{class_correct})")
        print(f"  Mean Position Error: {np.mean(position_errors_per_class[i])*table_size_m:.8f}m")
        print(f"  dx mean error: {np.mean(dx_mis_class_file_lst[i]):.12f} (normalized)")
        print(f"  dy mean error: {np.mean(dy_mis_class_file_lst[i]):.12f} (normalized)\n")
        class_dst_errors.append(np.mean(position_errors_per_class[i]) * table_size_m)

    print("Overall Euclidean Distance Error (m):", np.mean(class_dst_errors))

    # ==== Combined plot for all classes (with outlier removal and zoom) ====
    print("\n📈 Creating Combined Position Error Distribution Plot:")

    # Create figure with larger size for better readability
    plt.figure(figsize=(14, 8))

    # Plot histograms for all classes
    use_density = True
    alpha_value = 0.7  # Make histograms semi-transparent for overlap visibility

    # First pass: collect all errors to determine reasonable range
    all_errors = []
    class_error_data = []

    for i in range(num_classes):
        errors = np.array(position_errors_per_class[i]) * table_size_m  # Convert to meters
        if len(errors) == 0:
            print(f"Skipping empty class: {class_names[i]}")
            class_error_data.append(np.array([]))
            continue
        
        # Remove outliers using IQR method
        Q1 = np.percentile(errors, 25)
        Q3 = np.percentile(errors, 75)
        IQR = Q3 - Q1
        outlier_threshold = Q3 + 1.5 * IQR
        
        # Filter out outliers
        filtered_errors = errors[errors <= outlier_threshold]
        outliers_removed = len(errors) - len(filtered_errors)
        
        print(f"Class {class_names[i]}: {len(errors)} total, {outliers_removed} outliers removed, {len(filtered_errors)} kept")
        print(f"  Original range: {errors.min():.6f} - {errors.max():.6f}m")
        print(f"  Filtered range: {filtered_errors.min():.6f} - {filtered_errors.max():.6f}m")
        
        class_error_data.append(filtered_errors)
        all_errors.extend(filtered_errors)

    # Determine the x-axis range based on filtered data
    if all_errors:
        max_error = np.max(all_errors)
        x_limit = min(max_error * 1.1, 0.01)  # Cap at 1cm or 110% of max, whichever is smaller
        print(f"\nSetting x-axis limit to: {x_limit:.6f}m")
    else:
        x_limit = 0.005

    # Create shared bin edges for all histograms
    n_bins = 40
    bin_edges = np.linspace(0, x_limit, n_bins + 1)

    # Now create the histograms with the filtered data using step plots for better visibility
    for i in range(num_classes):
        filtered_errors = class_error_data[i]
        if len(filtered_errors) == 0:
            continue
        
        # Create histogram data
        hist_data, _ = np.histogram(filtered_errors, bins=bin_edges, density=use_density)
        bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
        
        # Plot as step function for cleaner overlapping
        plt.step(bin_centers, hist_data, where='mid', linewidth=2.5, alpha=0.9,
                color=class_colors[i], label=f"{class_names[i]} (n={len(filtered_errors)})")
        
        # Optional: add a faint filled area under the curve
        plt.fill_between(bin_centers, hist_data, alpha=0.15, color=class_colors[i], step='mid')
        
        print(f"Class {class_names[i]}: mean={np.mean(filtered_errors):.5f}m, std={np.std(filtered_errors):.5f}m")

    # Customize the plot
    ylabel = "Density" if use_density else "Frequency"
    plt.xlabel("Euclidean Distance Error (m)", fontsize=12)
    plt.ylabel(ylabel, fontsize=12)
    plt.title("Position Error Distribution: All Classes Combined (Step Plot)", fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')

    # Set x-axis limit to focus on main distribution
    plt.xlim(0, x_limit)

    # Add vertical line at threshold
    if threshold <= x_limit:
        plt.axvline(x=threshold, color='red', linestyle='--', linewidth=2, alpha=0.8, 
                label=f'Threshold ({threshold*1000:.1f}mm)')

    # Add statistics text box in top right
    total_samples = sum(len(data) for data in class_error_data)
    outliers_total = sum(len(position_errors_per_class[i]) for i in range(num_classes)) - total_samples

    stats_text = f"Threshold: {threshold*1000:.1f}mm\n"
    stats_text += f"Files processed: {file_counter}\n"
    stats_text += f"Total samples: {total_samples}\n"
    stats_text += f"Outliers removed: {outliers_total}\n"
    stats_text += f"X-axis limit: {x_limit*1000:.1f}mm"

    plt.text(0.98, 0.98, stats_text, transform=plt.gca().transAxes, 
            verticalalignment='top', horizontalalignment='right',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8),
            fontsize=10)

    # Save combined plot
    combined_filename = plots_save_path / "position_error_all_classes_combined_step.png"
    plt.tight_layout()
    plt.savefig(combined_filename, dpi=300, bbox_inches='tight')
    plt.close()

    print(f"✅ Saved combined step plot: {combined_filename}")

    # ==== Alternative: Create separate curves (kernel density estimation) ====
    print("\n📈 Creating Smooth Density Plot:")

    plt.figure(figsize=(14, 8))

    from scipy.stats import gaussian_kde

    for i in range(num_classes):
        filtered_errors = class_error_data[i]
        if len(filtered_errors) < 10:  # Need enough points for KDE
            continue
        
        # Create smooth density curve
        kde = gaussian_kde(filtered_errors)
        x_smooth = np.linspace(0, x_limit, 200)
        density_smooth = kde(x_smooth)
        
        plt.plot(x_smooth, density_smooth, linewidth=3, alpha=0.9,
                color=class_colors[i], label=f"{class_names[i]} (n={len(filtered_errors)})")
        
        # Add faint fill under curve
        plt.fill_between(x_smooth, density_smooth, alpha=0.2, color=class_colors[i])

    # Customize the plot
    plt.xlabel("Euclidean Distance Error (m)", fontsize=14)
    plt.ylabel("Density", fontsize=14)
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    # plt.title("Position Error Distribution: All Classes Combined (Smooth Density)", fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=18)

    # Set x-axis limit
    plt.xlim(0, x_limit)

    # Add vertical line at threshold
    if threshold <= x_limit:
        plt.axvline(x=threshold, color='red', linestyle='--', linewidth=2, alpha=0.8, 
                label=f'Threshold ({threshold*1000:.1f}mm)')

    # Add statistics text box in top right
    plt.text(0.98, 0.98, stats_text, transform=plt.gca().transAxes, 
            verticalalignment='top', horizontalalignment='right',
            bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8),
            fontsize=18)

    # Save smooth density plot
    smooth_filename = plots_save_path / "position_error_all_classes_smooth_density.png"
    plt.tight_layout()
    plt.savefig(smooth_filename, dpi=300, bbox_inches='tight')
    plt.close()

    print(f"✅ Saved smooth density plot: {smooth_filename}")

    # ==== Create violin plot as another alternative ====
    print("\n📈 Creating Violin Plot:")

    plt.figure(figsize=(12, 8))

    # Prepare data for violin plot
    violin_data = []
    violin_labels = []
    violin_colors_list = []

    for i in range(num_classes):
        filtered_errors = class_error_data[i]
        if len(filtered_errors) > 0:
            violin_data.append(filtered_errors * 1000)  # Convert to mm for better readability
            violin_labels.append(f"{class_names[i]}\n(n={len(filtered_errors)})")
            violin_colors_list.append(class_colors[i])

    # Create violin plot
    parts = plt.violinplot(violin_data, positions=range(len(violin_data)), showmeans=True, showmedians=True)

    # Color the violins
    for pc, color in zip(parts['bodies'], violin_colors_list):
        pc.set_facecolor(color)
        pc.set_alpha(0.7)

    # Customize other elements
    parts['cmeans'].set_color('red')
    parts['cmeans'].set_linewidth(2)
    parts['cmedians'].set_color('black')
    parts['cmedians'].set_linewidth(2)

    plt.xticks(range(len(violin_labels)), violin_labels, rotation=45)
    plt.ylabel("Euclidean Distance Error (mm)", fontsize=12)
    plt.title("Position Error Distribution: Violin Plot", fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)

    # Add horizontal line at threshold
    plt.axhline(y=threshold*1000, color='red', linestyle='--', alpha=0.7, 
            label=f'Threshold ({threshold*1000:.1f}mm)')
    plt.legend()

    # Save violin plot
    violin_filename = plots_save_path / "position_error_all_classes_violin.png"
    plt.tight_layout()
    plt.savefig(violin_filename, dpi=300, bbox_inches='tight')
    plt.close()

    print(f"✅ Saved violin plot: {violin_filename}")

    # ==== Create a zoomed subplot version as well ====
    print("\n📈 Creating Zoomed Subplot Version:")

    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.flatten()

    for i in range(num_classes):
        filtered_errors = class_error_data[i]
        if len(filtered_errors) == 0:
            axes[i].text(0.5, 0.5, f"No data\nfor {class_names[i]}", 
                        ha='center', va='center', transform=axes[i].transAxes)
            axes[i].set_title(f"{class_names[i]}")
            continue
        
        bin_count = min(25, max(8, len(filtered_errors) // 8))
        
        axes[i].hist(
            filtered_errors,
            bins=bin_count,
            alpha=0.75,
            density=use_density,
            color=class_colors[i],
            edgecolor='black'
        )
        
        # Set consistent x-axis for all subplots
        axes[i].set_xlim(0, x_limit)
        
        # Add threshold line if within range
        if threshold <= x_limit:
            axes[i].axvline(x=threshold, color='red', linestyle='--', alpha=0.7)
        
        axes[i].set_xlabel("Error (m)")
        axes[i].set_ylabel("Density" if use_density else "Frequency")
        axes[i].set_title(f"{class_names[i]}\n(n={len(filtered_errors)}, μ={np.mean(filtered_errors):.5f}m)")
        axes[i].grid(True, alpha=0.3)

    plt.suptitle("Position Error Distribution by Class (Zoomed, Outliers Removed)", fontsize=16, fontweight='bold')
    plt.tight_layout()

    # Save subplot version
    subplot_filename = plots_save_path / "position_error_all_classes_subplots_zoomed.png"
    plt.savefig(subplot_filename, dpi=300, bbox_inches='tight')
    plt.close()

    print(f"✅ Saved subplot version (zoomed): {subplot_filename}")

    # ==== Alternative: Subplot version (each class in separate subplot) ====
    print("\n📈 Creating Subplot Version:")

    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.flatten()

    for i in range(num_classes):
        errors = np.array(position_errors_per_class[i]) * table_size_m
        if len(errors) == 0:
            axes[i].text(0.5, 0.5, f"No data\nfor {class_names[i]}", 
                        ha='center', va='center', transform=axes[i].transAxes)
            axes[i].set_title(f"{class_names[i]}")
            continue
        
        axes[i].hist(
            errors,
            bins=30,
            alpha=0.75,
            density=use_density,
            color=class_colors[i],
            edgecolor='black'
        )
        
        axes[i].set_xlabel("Error (m)")
        axes[i].set_ylabel("Density" if use_density else "Frequency")
        axes[i].set_title(f"{class_names[i]}\n(n={len(errors)}, μ={np.mean(errors):.4f}m)")
        axes[i].grid(True, alpha=0.3)

    plt.suptitle("Position Error Distribution by Class", fontsize=16, fontweight='bold')
    plt.tight_layout()

    # Save subplot version
    subplot_filename = plots_save_path / "position_error_all_classes_subplots.png"
    plt.savefig(subplot_filename, dpi=300, bbox_inches='tight')
    plt.close()

    print(f"✅ Saved subplot version: {subplot_filename}")

    # ==== Box plot version for comparison ====
    print("\n📈 Creating Box Plot Version:")

    plt.figure(figsize=(12, 6))

    # Prepare data for box plot
    box_data = []
    box_labels = []
    box_colors_list = []

    for i in range(num_classes):
        errors = np.array(position_errors_per_class[i]) * table_size_m
        if len(errors) > 0:
            box_data.append(errors)
            box_labels.append(f"{class_names[i]}\n(n={len(errors)})")
            box_colors_list.append(class_colors[i])

    # Create box plot
    box_plot = plt.boxplot(box_data, labels=box_labels, patch_artist=True)

    # Color the boxes
    for patch, color in zip(box_plot['boxes'], box_colors_list):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)

    plt.ylabel("Euclidean Distance Error (m)", fontsize=12)
    plt.title("Position Error Distribution: Box Plot Comparison", fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.xticks(rotation=45)

    # Add horizontal line at threshold
    plt.axhline(y=threshold, color='red', linestyle='--', alpha=0.7, 
            label=f'Threshold ({threshold}m)')
    plt.legend()

    # Save box plot
    boxplot_filename = plots_save_path / "position_error_all_classes_boxplot.png"
    plt.tight_layout()
    plt.savefig(boxplot_filename, dpi=300, bbox_inches='tight')
    plt.close()

    print(f"✅ Saved box plot: {boxplot_filename}")

    print(f"\n🎯 All plots saved to: {plots_save_path}")
    print("📊 Generated plots:")
    print(f"  1. Combined histogram: {combined_filename.name}")
    print(f"  2. Subplot version: {subplot_filename.name}")
    print(f"  3. Box plot comparison: {boxplot_filename.name}")


if __name__ == "__main__":
    levels = ["easy", "medium", "hard"]
    for level in levels:
        main(level)