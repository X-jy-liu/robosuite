import json
import math
import re
from pathlib import Path
from matplotlib import pyplot as plt

scene_06_mapping = {
    "obj0": "blue cylinder",
    "obj1": "green cube",
    "obj2": "red cube",
    "obj3": "green cube",
    "obj4": "green cylinder"
}

# Define your vocabulary
colors = ["red", "green", "blue"]
shapes = ["cube", "cylinder"]

# Extend shapes to include plural forms
shape_variants = shapes + [s + "s" for s in shapes]  # ['cube', 'cylinder', 'cubes', 'cylinders']

def extract_object_phrases_plural(command, colors, shape_variants):
    pattern = r"\b(" + "|".join(colors) + r")\s+(" + "|".join(shape_variants) + r")\b"
    return [(match.group(0), match.start()) for match in re.finditer(pattern, command)]

def normalize_shape(shape):
    return shape[:-1] if shape.endswith('s') else shape

def get_xy(pos3d):
    return pos3d[0], pos3d[1]

def euclidean_distance(pos_a, pos_b):
    return math.sqrt((pos_a[0] - pos_b[0])**2 + (pos_a[1] - pos_b[1])**2)

# Load your JSON file
log_dir = Path("/home/jingyang/robosuite/myCode/my_planning_app/logs/scene_06/experiments_logging/")
log_files = list(log_dir.glob("*.json"))

dist_lst = [] # each element it a tuple(init_distance, mid_distance, final_distance)
for i, log_path in enumerate(log_files):
    print(f"Experiment{i+1} from {log_path.name}:")
    dist_cache = []
    with open(log_path, "r") as f:
        data = json.load(f)
    command = log_path.stem.split("_")[1:-2]
    command = " ".join(command).replace("_", " ")
    phrases_with_pos = extract_object_phrases_plural(command, colors, shape_variants)
    print(f"Matched phrases with position: {phrases_with_pos}")

    # Normalize and match against mapping
    normalized_phrases = [(' '.join([c, normalize_shape(s)]), pos)
                        for phrase, pos in phrases_with_pos
                        for c, s in [phrase.split()]]

    # Find all objects that match the normalized description
    matching_obj_ids = []
    for phrase, _ in normalized_phrases:
        for obj_id, label in scene_06_mapping.items():
            if label == phrase:
                matching_obj_ids.append(obj_id)

    obj1_id_1, obj_id_2 = matching_obj_ids[0], matching_obj_ids[1]

    for i, step in enumerate(data["steps"]):
        step_type = "Initial" if i == 0 else "Follow-up"
        init_pos = step["init_obj"]
        # Print the initial distances for obj2 and obj4
        obj1_init = get_xy(init_pos[obj1_id_1]["position"])
        obj2_init = get_xy(init_pos[obj_id_2]["position"])
        init_distance = euclidean_distance(obj1_init, obj2_init)
        dist_cache.append(init_distance)
        # Final positions
        obj_pos_history = step["obj_pos_history"]
        obj1_final = get_xy(obj_pos_history[-1][obj1_id_1]["position"])
        obj2_final = get_xy(obj_pos_history[-1][obj_id_2]["position"])
        final_distance = euclidean_distance(obj1_final, obj2_final)
        dist_cache.append(final_distance)
        if len(dist_cache) == 4:
            dist_lst.append((dist_cache[0],dist_cache[1],dist_cache[3]))
        print(f"{step_type} command:")
        print(f"  Initial distance between interested objects: {init_distance:.4f}")
        print(f"  Final distance between interested objects:   {final_distance:.4f}\n")

# Labels
experiment_labels = [f"Exp{i+1}" for i in range(len(dist_lst))]
initials, mids, finals = zip(*dist_lst)
mid_pct = [(1 - m / i) * 100 for i, m, _ in dist_lst]
final_pct = [(1 - f / i) * 100 for i, _, f in dist_lst]

# Plotting
x = range(len(dist_lst))
width = 0.25

plt.figure(figsize=(12, 6))
bars1 = plt.bar([i - width for i in x], initials, width=width, label="Initial", color="#FDB813")
bars2 = plt.bar(x, mids, width=width, label="Mid", color='#F05E23')
bars3 = plt.bar([i + width for i in x], finals, width=width, label="Final", color='#EC1C4B')

# Annotate mid and final bars with percentage reduction
for i in range(len(x)):
    plt.text(x[i], mids[i] + 0.005, f"-{mid_pct[i]:.1f}%", ha='center', va='bottom', fontsize=9, color='orange')
    plt.text(x[i] + width, finals[i] + 0.005, f"-{final_pct[i]:.1f}%", ha='center', va='bottom', fontsize=9, color='green')

plt.xticks(ticks=x, labels=experiment_labels)
plt.ylabel("Distance between objects")
plt.title("Distance Reduction Across Ambiguous Commands (with % decrease)")
plt.legend()
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.tight_layout()
# Save the plot
save_dir = Path("/home/jingyang/robosuite/myCode/my_planning_app/logs/")
output_path =  save_dir / "ambiguous_performance_plot.png"
plt.savefig(output_path, dpi=300, bbox_inches='tight')
plt.show()
