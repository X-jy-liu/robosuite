import json
import math
from pathlib import Path
from matplotlib import pyplot as plt
import numpy as np

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

def interpolate_noisy_distances(dist_lst, steps_per_segment=5, noise_std=0.01):
    all_sequences = []
    for init, mid, final in dist_lst:
        reds = np.concatenate([
            np.linspace(init[0], mid[0], steps_per_segment, endpoint=False),
            np.linspace(mid[0], final[0], steps_per_segment)
        ])
        greens = np.concatenate([
            np.linspace(init[1], mid[1], steps_per_segment, endpoint=False),
            np.linspace(mid[1], final[1], steps_per_segment)
        ])
        red_noisy = reds + np.random.normal(0, noise_std, size=reds.shape)
        green_noisy = greens + np.random.normal(0, noise_std, size=greens.shape)
        seq = list(zip(red_noisy, green_noisy))
        all_sequences.append(seq)
    return all_sequences

def convert_sequence_to_ratios(dist_seq):
    return [[g / r if r != 0 else float('inf') for r, g in seq] for seq in dist_seq]

def classify_sequence_success(dist_lst):
    success_flags = []
    for (r0, g0), (r1, g1), (r2, g2) in dist_lst:
        def safe_ratio(g, r): return g / r if r != 0 else float('inf')
        r0 = safe_ratio(g0, r0)
        r1 = safe_ratio(g1, r1)
        r2 = safe_ratio(g2, r2)
        success_flags.append(r0 > r1 > r2)
    return success_flags

def plot_sequence_distances(dist_seq, if_save=False):
    plt.figure(figsize=(12, 6))
    for i, seq in enumerate(dist_seq):
        red_curve = np.array([-r for r, g in seq])
        green_curve = np.array([g for r, g in seq])
        x_vals = np.linspace(0, 1, len(seq))
        plt.plot(x_vals, green_curve, color='green', alpha=0.6, label='Distance betwee Blue Cube and Green Cylinder' if i == 0 else "")
        plt.plot(x_vals, red_curve, color='red', alpha=0.6, label='Distance between Blue Cube and Red Cylinder' if i == 0 else "")
    plt.axhline(0, color='black', linewidth=0.8)
    plt.ylabel("Distance (Green ↑, Red ↓)")
    plt.xlabel("Normalized Time")
    plt.title("Noisy Linear Distances from Blue Cube to Red/Green Cylinders")
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()
    if if_save:
        save_dir = Path("/home/jingyang/robosuite/myCode/my_planning_app/logs/plots")
        save_path = save_dir / "smoothed_distances_plot_with_noise_interpolation.png"
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print("Plot saved as 'smoothed_distances_plot_with_noise_interpolation.png'.")
    plt.show()

def plot_sequence_ratios(ratio_seq, success_flags=None, if_save=False):
    plt.figure(figsize=(12, 6))
    counter = 0
    for i, seq in enumerate(ratio_seq):
        x_vals = np.linspace(0, 1, len(seq))
        if success_flags is None or success_flags[i]:
            plt.plot(x_vals, seq, color='blue', alpha=0.7, label='Success Experiments' if i == 0 else "")
        else:
            plt.plot(x_vals, seq, color='black', linestyle='--', linewidth=1, label='Failed Experiments' if counter == 0 else "")
            counter += 1
    plt.axhline(1.0, linestyle='--', color='gray', linewidth=0.8)
    plt.ylabel("Green : Red Distance Ratio",fontsize=16)
    plt.xlabel("Normalized Time",fontsize=16)
    # plt.title("Smoothed Ratio of Distances (Green / Red) with Failure Highlighting")
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend(fontsize=14,loc='upper left')
    plt.tight_layout()
    if if_save:
        plt.savefig("smoothed_ratios_with_failures_plot.png", dpi=300, bbox_inches='tight')
        print("Plot saved as 'smoothed_ratios_with_failures_plot.png'.")
    plt.show()

if __name__ == "__main__":
    log_dir = Path("/home/jingyang/robosuite/myCode/my_planning_app/logs/scene_07/experiments_logging/")
    log_files = list(log_dir.glob("*.json"))

    dist_lst = []
    for log_path in log_files:
        with open(log_path, "r") as f:
            data = json.load(f)
        dist_cache = []
        for step in data["steps"]:
            init = step["init_obj"]
            obj_blue = get_xy(init["obj0"]["position"])
            obj_red = get_xy(init["obj1"]["position"])
            obj_green = get_xy(init["obj2"]["position"])
            init_dist_red = euclidean_distance(obj_blue, obj_red)
            init_dist_green = euclidean_distance(obj_blue, obj_green)
            dist_cache.append((init_dist_red, init_dist_green))
            last = step["obj_pos_history"][-1]
            obj_blue_f = get_xy(last["obj0"]["position"])
            obj_red_f = get_xy(last["obj1"]["position"])
            obj_green_f = get_xy(last["obj2"]["position"])
            final_dist_red = euclidean_distance(obj_blue_f, obj_red_f)
            final_dist_green = euclidean_distance(obj_blue_f, obj_green_f)
            dist_cache.append((final_dist_red, final_dist_green))
            if len(dist_cache) == 4:
                dist_lst.append((dist_cache[0], dist_cache[1], dist_cache[3]))

    success_flags = []
    failed_paths = []

    for idx, ((r0, g0), (r1, g1), (r2, g2)) in enumerate(dist_lst):
        def safe_ratio(g, r): return g / r if r != 0 else float('inf')
        ratio_0 = safe_ratio(g0, r0)
        ratio_1 = safe_ratio(g1, r1)
        ratio_2 = safe_ratio(g2, r2)
        is_success = ratio_0 > ratio_1 > ratio_2
        success_flags.append(is_success)
        if not is_success and ratio_2 > ratio_1:
            failed_paths.append(log_files[idx])

    print("Files where third ratio > second ratio:")
    for p in failed_paths:
        print(p)
    dist_seq = interpolate_noisy_distances(dist_lst,steps_per_segment=10, noise_std=0)
    ratio_seq = convert_sequence_to_ratios(dist_seq)

    print(f"Success rate: {sum(success_flags) / len(success_flags):.2%}")
    print(f"Number of successful experiments: {sum(success_flags)} out of {len(success_flags)}")
    print(f"Success flags: {success_flags}")

    plot_sequence_distances(dist_seq)
    plot_sequence_ratios(ratio_seq, success_flags, if_save=True)
