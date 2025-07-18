import json
import math
from pathlib import Path
from matplotlib import pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline

# Mapping for clarity (not used directly in code below)
scene_07_mapping = {
    "obj0": "blue cube",
    "obj1": "red cylinder",
    "obj2": "green cylinder"
}

def get_xy(pos3d):
    return pos3d[0], pos3d[1]

def euclidean_distance(pos_a, pos_b):
    return math.sqrt((pos_a[0] - pos_b[0])**2 + (pos_a[1] - pos_b[1])**2)

# Directory containing JSON logs
log_dir = Path("/home/jingyang/robosuite/myCode/my_planning_app/logs/scene_07/experiments_logging/")
log_files = list(log_dir.glob("*.json"))

dist_lst = []  # List[Tuple[(init), (mid), (final)]]

def plot_smoothed_distances_aligned(dist_lst, steps_per_segment=5, noise_std = 0.01, if_save=False):
    plt.figure(figsize=(12, 6))
    
    total_steps = steps_per_segment * 2
    x_vals = np.linspace(0, 1, total_steps)  # normalized time

    for i, (initial, mid, final) in enumerate(dist_lst):
        red_curve = np.concatenate([
            np.linspace(initial[0], mid[0], steps_per_segment, endpoint=False),
            np.linspace(mid[0], final[0], steps_per_segment)
        ])
        green_curve = np.concatenate([
            np.linspace(initial[1], mid[1], steps_per_segment, endpoint=False),
            np.linspace(mid[1], final[1], steps_per_segment)
        ])
        
        green_shifted = green_curve
        red_shifted = -1 * red_curve
        x_smooth = np.linspace(0, 1, 100)
        green_spline = make_interp_spline(x_vals, green_shifted, k=2)(x_smooth)
        red_spline = make_interp_spline(x_vals, red_shifted, k=2)(x_smooth)

        plt.plot(x_smooth, green_spline, color='green', alpha=0.6)
        plt.plot(x_smooth, red_spline, color='red', alpha=0.6)

    plt.axhline(0, color='black', linewidth=0.8)
    plt.ylabel("Distance (Green ↑, Red ↓)")
    plt.xlabel("Normalized Time")
    # plt.title("Smoothed Distances from Blue Cube to Red/Green Cylinders")
    plt.grid(True, linestyle='--', alpha=0.5)

    yticks = np.arange(-0.4, 0.45, 0.05)
    ytick_labels = [f"{abs(y):.2f}" for y in yticks]
    plt.yticks(yticks, ytick_labels)

    plt.tight_layout()
    if if_save:
        saving_dir = Path("/home/jingyang/robosuite/myCode/my_planning_app/logs/plots")
        saving_path = saving_dir / "smoothed_distances_plot.png"
        plt.savefig(saving_path, dpi=300, bbox_inches='tight')
        print("Plot saved as 'smoothed_distances_plot.png'.")
    plt.show()

def plot_smoothed_ratios_aligned(ratio_lst, steps_per_segment=5, if_save=False):
    """
    Plot green:red distance ratios (single values) with smoothing.
    Input format:
        ratio_lst = [((init,), (mid,), (final,)), ...]
    """
    plt.figure(figsize=(12, 6))

    total_steps = steps_per_segment * 2
    x_vals = np.linspace(0, 1, total_steps)

    for i, (init, mid, final) in enumerate(ratio_lst):
        ratio_curve = np.concatenate([
            np.linspace(init[0], mid[0], steps_per_segment, endpoint=False),
            np.linspace(mid[0], final[0], steps_per_segment)
        ])

        # Smoothing with spline
        x_smooth = np.linspace(0, 1, 100)
        ratio_spline = make_interp_spline(x_vals, ratio_curve, k=2)(x_smooth)

        plt.plot(x_smooth, ratio_spline, alpha=0.7)

    plt.ylabel("Green : Red Distance Ratio")
    plt.xlabel("Normalized Time")
    # plt.title("Smoothed Ratio of Distances (Green / Red)")
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.axhline(1.0, linestyle='--', color='black', linewidth=0.8)  # Reference line: equal distance
    plt.tight_layout()
    if if_save:
        saving_dir = Path("/home/jingyang/robosuite/myCode/my_planning_app/logs/plots")
        saving_path = saving_dir / "smoothed_ratios_plot.png"
        plt.savefig(saving_path, dpi=300, bbox_inches='tight')
        print("Plot saved as 'smoothed_ratios_plot.png'.")
    plt.show()
    
def plot_smoothed_ratios_aligned_with_failures(ratio_lst, success_flags, steps_per_segment=5, if_save=False):
    """
    Plot green:red distance ratios (single values) with smoothing.
    Failed experiments are highlighted in black.
    """
    plt.figure(figsize=(12, 6))

    total_steps = steps_per_segment * 2
    x_vals = np.linspace(0, 1, total_steps)

    for i, ((init,), (mid,), (final,)) in enumerate(ratio_lst):
        ratio_curve = np.concatenate([
            np.linspace(init, mid, steps_per_segment, endpoint=False),
            np.linspace(mid, final, steps_per_segment)
        ])
        x_smooth = np.linspace(0, 1, 100)
        ratio_spline = make_interp_spline(x_vals, ratio_curve, k=2)(x_smooth)

        if success_flags[i]:
            plt.plot(x_smooth, ratio_spline, color='blue', alpha=0.7)
        else:
            plt.plot(x_smooth, ratio_spline, color='black', linestyle='--', linewidth=2)

    plt.axhline(1.0, linestyle='--', color='gray', linewidth=0.8)
    plt.ylabel("Green : Red Distance Ratio")
    plt.xlabel("Normalized Time")
    # plt.title("Smoothed Ratio of Distances (Green / Red) with Failure Highlighting")
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()
    if if_save:
        saving_dir = Path("/home/jingyang/robosuite/myCode/my_planning_app/logs/plots")
        saving_path = saving_dir / "smoothed_ratios_with_failures_plot.png"
        plt.savefig(saving_path, dpi=300, bbox_inches='tight')
        print("Plot saved as 'smoothed_ratios_with_failures_plot.png'.")
    plt.show()

def calculate_success_rate_from_ratio(dist_lst):
    """
    Success = green/red ratio strictly decreases from init → mid → final
    Input:
        dist_lst = [((red_init, green_init), (red_mid, green_mid), (red_final, green_final)), ...]
    Returns:
        success_rate: float between 0 and 1
        success_flags: list of booleans for each experiment
    """
    success_flags = []

    for (r0, g0), (r1, g1), (r2, g2) in dist_lst:
        def safe_ratio(g, r):
            return g / r if r != 0 else float('inf')

        ratio0 = safe_ratio(g0, r0)
        ratio1 = safe_ratio(g1, r1)
        ratio2 = safe_ratio(g2, r2)

        success = ratio0 > ratio1 > ratio2
        success_flags.append(success)

    success_rate = sum(success_flags) / len(success_flags)
    return success_rate, success_flags

def convert_distances_to_ratios(dist_lst_abs):
    """
    Convert absolute distance tuples into green:red ratio tuples.

    Input format:
        dist_lst_abs = [((red_init, green_init), (red_mid, green_mid), (red_final, green_final)), ...]

    Output format:
        dist_lst_ratios = [((init_ratio), (mid_ratio), (final_ratio)), ...]
                          where each ratio = green / red
    """
    ratio_lst = []

    for (red_init, green_init), (red_mid, green_mid), (red_final, green_final) in dist_lst_abs:
        def safe_div(n, d):
            return n / d if d != 0 else float('inf')  # Avoid divide-by-zero

        init_ratio = safe_div(green_init, red_init)
        mid_ratio = safe_div(green_mid, red_mid)
        final_ratio = safe_div(green_final, red_final)

        ratio_lst.append(((init_ratio,), (mid_ratio,), (final_ratio,)))

    return ratio_lst

if __name__ == "__main__":
    for i, log_path in enumerate(log_files):
        print(f"\nExperiment {i+1} from {log_path.name}:")
        dist_cache = []
        
        with open(log_path, "r") as f:
            data = json.load(f)
        
        command = log_path.stem.split("_")[1:-2]
        command = " ".join(command).replace("_", " ")

        for step_idx, step in enumerate(data["steps"]):
            step_type = "Initial" if step_idx == 0 else "Follow-up"
            init_pos = step["init_obj"]

            # Use correct object index:
            obj_blue = get_xy(init_pos["obj0"]["position"])  # blue cube
            obj_red = get_xy(init_pos["obj1"]["position"])   # red cylinder
            obj_green = get_xy(init_pos["obj2"]["position"]) # green cylinder

            init_dist_red = euclidean_distance(obj_blue, obj_red)
            init_dist_green = euclidean_distance(obj_blue, obj_green)
            dist_cache.append((init_dist_red, init_dist_green))

            # Final positions (last in obj_pos_history)
            obj_pos_history = step["obj_pos_history"]
            last_pos = obj_pos_history[-1]

            obj_blue_final = get_xy(last_pos["obj0"]["position"])
            obj_red_final = get_xy(last_pos["obj1"]["position"])
            obj_green_final = get_xy(last_pos["obj2"]["position"])

            final_dist_red = euclidean_distance(obj_blue_final, obj_red_final)
            final_dist_green = euclidean_distance(obj_blue_final, obj_green_final)
            dist_cache.append((final_dist_red, final_dist_green))

            if len(dist_cache) == 4:
                # Tuple: (initial, mid, final)
                dist_lst.append((dist_cache[0], dist_cache[1], dist_cache[3])) # List[Tuple[float, float]] [0] is the intial distances between obj1 and obj2, obj1 and obj3; [1] is the mid distances; [3] is the final distances

            print(f"{step_type} command:\n{step['task_command']}")
            print(f"  Initial distance to red cylinder:   {init_dist_red:.4f}")
            print(f"  Initial distance to green cylinder: {init_dist_green:.4f}")
            print("  ------------------------------------")
            print(f"  Final distance to red cylinder:     {final_dist_red:.4f}")
            print(f"  Final distance to green cylinder:   {final_dist_green:.4f}")

    success_rate, success_flags = calculate_success_rate_from_ratio(dist_lst)
    print(f"\nSuccess rate: {success_rate:.2%}")
    print(f"Success flags: {success_flags}")
    plot_smoothed_distances_aligned(dist_lst,if_save=True)
    plot_smoothed_ratios_aligned(convert_distances_to_ratios(dist_lst), if_save=True)
    plot_smoothed_ratios_aligned_with_failures(convert_distances_to_ratios(dist_lst), success_flags,if_save=True)