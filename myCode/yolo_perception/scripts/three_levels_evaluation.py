from pathlib import Path
import numpy as np
from tqdm import tqdm
from collections import defaultdict
import matplotlib.pyplot as plt
import os
from scipy.optimize import linear_sum_assignment
from scipy.stats import gaussian_kde

def evaluate_noise_level(ref_dir, pred_dir, plots_save_path, noise_level, 
                        threshold=0.0025, table_size_m=0.8, num_classes=6):
    """
    Evaluate predictions for a specific noise level
    
    Args:
        ref_dir (Path): Directory containing reference labels
        pred_dir (Path): Directory containing prediction labels for this noise level
        plots_save_path (Path): Directory to save plots
        noise_level (str): Name of the noise level (e.g., 'easy', 'medium', 'hard')
        threshold (float): Distance threshold in meters
        table_size_m (float): Table size in meters
        num_classes (int): Number of object classes
    
    Returns:
        dict: Evaluation results for this noise level
    """
    
    # Normalize threshold
    norm_threshold = threshold / table_size_m
    print(f"\n{'='*60}")
    print(f"Evaluating {noise_level.upper()} noise level")
    print(f"{'='*60}")
    print(f"Threshold in meters: {threshold}")
    print(f"Normalized threshold: {norm_threshold:.6f} over table size {table_size_m}m")
    
    class_names = [
        "red_cube", "green_cube", "blue_cube",
        "red_cylinder", "green_cylinder", "blue_cylinder"
    ]
    
    class_colors = [
        '#FF4444', '#44FF44', '#4444FF',  # cubes - red, green, blue
        '#FF8888', '#88FF88', '#8888FF'   # cylinders - light versions
    ]
    
    # Initialize stats
    total_gt_per_class = np.zeros(num_classes, dtype=int)
    correct_class_pred_per_class = np.zeros(num_classes, dtype=int)
    correct_position_pred_per_class = np.zeros(num_classes, dtype=int)
    position_errors_per_class = defaultdict(list)
    dx_mis_class_file_lst = defaultdict(list)
    dy_mis_class_file_lst = defaultdict(list)
    
    mismatch_files = []
    file_counter = 0
    ref_txt_files = list(ref_dir.glob("*.txt"))
    
    print(f"Found {len(ref_txt_files)} reference files.")
    
    # Process each file
    for ref_file in tqdm(ref_txt_files, desc=f"Evaluating {noise_level} predictions"):
        pred_file = pred_dir / ref_file.name
        if not pred_file.exists():
            continue
        else:
            file_counter += 1
            
        with open(ref_file) as f1, open(pred_file) as f2:
            ref_lines = [line.strip().split() for line in f1.readlines() if line.strip()]
            pred_lines = [line.strip().split() for line in f2.readlines() if line.strip()]
        
        if not ref_lines:
            continue
            
        # Handle case where no predictions were made
        if not pred_lines:
            for ref_line in ref_lines:
                class_r = int(ref_line[0])
                total_gt_per_class[class_r] += 1
            continue
        
        # Extract (x, y) positions only
        ref_xy = np.array([[float(x), float(y)] for _, x, y, _, _ in ref_lines])
        pred_xy = np.array([[float(x), float(y)] for _, x, y, _, _ in pred_lines])
        
        # Compute pairwise Euclidean distances
        dists = np.linalg.norm(ref_xy[:, None, :] - pred_xy[None, :, :], axis=2)
        
        # Hungarian algorithm to find best match
        row_ind, col_ind = linear_sum_assignment(dists)
        
        # Process matched pairs
        for i, j in zip(row_ind, col_ind):
            r_line = ref_lines[i]
            p_line = pred_lines[j]
            
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
        
        # Handle unmatched ground truth objects (missed detections)
        unmatched_gt = set(range(len(ref_lines))) - set(row_ind)
        for i in unmatched_gt:
            class_r = int(ref_lines[i][0])
            total_gt_per_class[class_r] += 1
    
    print(f"Processed {file_counter} files for {noise_level} level.")
    print(f"Found {len(mismatch_files)} mismatches.")
    
    # Calculate and print results
    print(f"\n📊 {noise_level.upper()} Level - Class-wise Evaluation Report:")
    class_dst_errors = []
    results = {
        'noise_level': noise_level,
        'class_accuracies': {},
        'position_accuracies': {},
        'mean_errors': {},
        'total_gt': {},
        'correct_class': {},
        'correct_position': {}
    }
    
    for i in range(num_classes):
        name = class_names[i]
        total = total_gt_per_class[i]
        class_correct = correct_class_pred_per_class[i]
        pos_correct = correct_position_pred_per_class[i]
        
        class_acc = class_correct / total if total > 0 else 0
        pos_acc = pos_correct / class_correct if class_correct > 0 else 0
        mean_error = np.mean(position_errors_per_class[i]) * table_size_m if position_errors_per_class[i] else 0
        
        print(f"Class: {name}")
        print(f"  Classification Accuracy: {class_acc:.2%} ({class_correct}/{total})")
        print(f"  Position Accuracy (within {threshold}m): {pos_acc:.2%} ({pos_correct}/{class_correct})")
        print(f"  Mean Position Error: {mean_error:.6f}m")
        
        # Store results
        results['class_accuracies'][name] = class_acc
        results['position_accuracies'][name] = pos_acc
        results['mean_errors'][name] = mean_error
        results['total_gt'][name] = total
        results['correct_class'][name] = class_correct
        results['correct_position'][name] = pos_correct
        
        if position_errors_per_class[i]:
            class_dst_errors.append(mean_error)
    
    overall_error = np.mean(class_dst_errors) if class_dst_errors else 0
    print(f"\nOverall Euclidean Distance Error: {overall_error:.6f}m")
    results['overall_error'] = overall_error
    
    # Create plots for this noise level
    create_noise_level_plots(position_errors_per_class, class_names, class_colors, 
                           plots_save_path, noise_level, threshold, table_size_m, 
                           file_counter)
    
    return results

def create_noise_level_plots(position_errors_per_class, class_names, class_colors, 
                           plots_save_path, noise_level, threshold, table_size_m, file_counter):
    """Create plots for a specific noise level"""
    
    # Prepare data for plotting
    all_errors = []
    class_error_data = []
    
    for i in range(len(class_names)):
        errors = np.array(position_errors_per_class[i]) * table_size_m
        if len(errors) == 0:
            class_error_data.append(np.array([]))
            continue
        
        # Remove outliers using IQR method
        Q1 = np.percentile(errors, 25)
        Q3 = np.percentile(errors, 75)
        IQR = Q3 - Q1
        outlier_threshold = Q3 + 1.5 * IQR
        filtered_errors = errors[errors <= outlier_threshold]
        
        class_error_data.append(filtered_errors)
        all_errors.extend(filtered_errors)
    
    if not all_errors:
        print(f"No error data to plot for {noise_level} level")
        return
    
    # Determine plotting range
    max_error = np.max(all_errors)
    x_limit = min(max_error * 1.1, 0.01)  # Cap at 1cm
    
    # Create smooth density plot
    plt.figure(figsize=(14, 8))
    
    for i in range(len(class_names)):
        filtered_errors = class_error_data[i]
        if len(filtered_errors) < 10:
            continue
        
        # Create smooth density curve
        kde = gaussian_kde(filtered_errors)
        x_smooth = np.linspace(0, x_limit, 200)
        density_smooth = kde(x_smooth)
        
        plt.plot(x_smooth, density_smooth, linewidth=3, alpha=0.9,
                color=class_colors[i], label=f"{class_names[i]} (n={len(filtered_errors)})")
        plt.fill_between(x_smooth, density_smooth, alpha=0.2, color=class_colors[i])
    
    # Customize plot
    plt.xlabel("Euclidean Distance Error (m)", fontsize=14)
    plt.ylabel("Density", fontsize=14)
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    plt.title(f"Position Error Distribution - {noise_level.upper()} Level", fontsize=16, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=12)
    plt.xlim(0, x_limit)
    
    # Add threshold line
    if threshold <= x_limit:
        plt.axvline(x=threshold, color='red', linestyle='--', linewidth=2, alpha=0.8)
    
    # Add statistics
    total_samples = sum(len(data) for data in class_error_data)
    stats_text = f"Level: {noise_level.upper()}\n"
    stats_text += f"Threshold: {threshold*1000:.1f}mm\n"
    stats_text += f"Files processed: {file_counter}\n"
    stats_text += f"Total samples: {total_samples}"
    
    plt.text(0.98, 0.98, stats_text, transform=plt.gca().transAxes,
            verticalalignment='top', horizontalalignment='right',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8),
            fontsize=12)
    
    # Save plot
    plot_filename = plots_save_path / f"position_error_{noise_level}_level.png"
    plt.tight_layout()
    plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"✅ Saved {noise_level} level plot: {plot_filename}")

def create_comparison_plots(all_results, plots_save_path, threshold):
    """Create comparison plots across all noise levels"""
    
    noise_levels = [result['noise_level'] for result in all_results]
    class_names = list(all_results[0]['class_accuracies'].keys())
    
    # 1. Classification Accuracy Comparison
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    
    # Classification accuracy
    x = np.arange(len(class_names))
    width = 0.25
    
    for i, result in enumerate(all_results):
        class_accs = [result['class_accuracies'][name] for name in class_names]
        ax1.bar(x + i*width, class_accs, width, label=result['noise_level'].title(), alpha=0.8)
    
    ax1.set_xlabel('Object Classes')
    ax1.set_ylabel('Classification Accuracy')
    ax1.set_title('Classification Accuracy by Noise Level')
    ax1.set_xticks(x + width)
    ax1.set_xticklabels(class_names, rotation=45)
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Position accuracy
    for i, result in enumerate(all_results):
        pos_accs = [result['position_accuracies'][name] for name in class_names]
        ax2.bar(x + i*width, pos_accs, width, label=result['noise_level'].title(), alpha=0.8)
    
    ax2.set_xlabel('Object Classes')
    ax2.set_ylabel('Position Accuracy')
    ax2.set_title(f'Position Accuracy by Noise Level (within {threshold*1000:.1f}mm)')
    ax2.set_xticks(x + width)
    ax2.set_xticklabels(class_names, rotation=45)
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(plots_save_path / "accuracy_comparison_by_noise_level.png", dpi=300, bbox_inches='tight')
    plt.close()
    
    # 2. Overall Error Comparison
    plt.figure(figsize=(10, 6))
    overall_errors = [result['overall_error'] for result in all_results]
    colors = ['green', 'orange', 'red']
    
    bars = plt.bar(noise_levels, overall_errors, color=colors, alpha=0.7)
    plt.xlabel('Noise Level')
    plt.ylabel('Mean Position Error (m)')
    plt.title('Overall Position Error by Noise Level')
    plt.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for bar, error in zip(bars, overall_errors):
        plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.0001,
                f'{error:.4f}m', ha='center', va='bottom')
    
    plt.tight_layout()
    plt.savefig(plots_save_path / "overall_error_comparison.png", dpi=300, bbox_inches='tight')
    plt.close()
    
    print("✅ Saved comparison plots")

def main():
    """Main function to evaluate all noise levels"""
    
    # Configuration
    ref_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/labels")
    base_pred_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/inference_from_mini_train_3_levels_realistic_noise")
    plots_save_path = Path("/home/s2644572/robosuite/myCode/yolo_perception/plots_realistic_noise_comparison/")

    # Create output directory
    os.makedirs(plots_save_path, exist_ok=True)
    
    # Parameters
    threshold = 0.001  # meters
    table_size_m = 0.8
    num_classes = 6
    noise_levels = ['easy', 'medium', 'hard']
    
    # Evaluate each noise level
    all_results = []
    
    for noise_level in noise_levels:
        pred_dir = base_pred_dir / noise_level / "labels"
        
        if not pred_dir.exists():
            print(f"Warning: Prediction directory not found: {pred_dir}")
            continue
        
        result = evaluate_noise_level(
            ref_dir=ref_dir,
            pred_dir=pred_dir,
            plots_save_path=plots_save_path,
            noise_level=noise_level,
            threshold=threshold,
            table_size_m=table_size_m,
            num_classes=num_classes
        )
        all_results.append(result)
    
    # Create comparison plots
    if len(all_results) > 1:
        create_comparison_plots(all_results, plots_save_path, threshold)
    
    # Print summary
    print(f"\n{'='*80}")
    print("SUMMARY ACROSS ALL NOISE LEVELS")
    print(f"{'='*80}")
    
    for result in all_results:
        level = result['noise_level']
        error = result['overall_error']
        print(f"{level.upper()} Level - Overall Error: {error:.6f}m")
    
    print(f"\n✅ All evaluations completed. Results saved to: {plots_save_path}")

if __name__ == "__main__":
    main()