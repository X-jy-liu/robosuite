import json
import matplotlib.pyplot as plt
from pathlib import Path

def trajecotry_extractor(log_path, interested_objs):
    with open(log_path, "r") as f:
        data = json.load(f)
    
    init_objects = data.get("steps", [])[0].get("init_obj", {})
    obj_mapping = _build_object_description_mapping(init_objects)  # {'obj0': 'green cube', ...}
    inversed_obj_mapping = {v: k for k, v in obj_mapping.items()}  # {'green cube': 'obj0', ...}
    obj_names = [inversed_obj_mapping[desc] for desc in interested_objs]

    # extract trajectory data
    interested_pos_history = {obj_name: [] for obj_name in obj_names}
    for step in data.get("steps", []):
        for snapshot in step.get("obj_pos_history", []):
            for obj_name in obj_names:
                if obj_name in snapshot:
                    interested_pos_history[obj_name].append(snapshot[obj_name]['position'])

    return init_objects, interested_pos_history

def plot_trajectory(init_objects, generated_dots_path, file_name, interested_pos_history):
    """
    Plots the initial object positions and their trajectories.
    
    Args:
        init_objects (dict): Dictionary mapping object IDs (e.g., "obj0") to attributes.
        generated_dots (dict): {"reference_points": Dict of reference point name and position[x,y]}
        interested_pos_history (dict): Dict mapping object IDs to list of [x, y, z] positions over time.
    """
    fig, ax = plt.subplots(figsize=(8, 8))

    # Plot initial positions of all objects
    for obj_id, props in init_objects.items():
        y, x, _ = props['position']
        color = props['color']
        shape = props['shape']
        label = f"{color} {shape}"
        
        # Use different markers for shape
        marker = 's' if shape == 'cube' else 'o'
        ax.scatter(x, y, c=color, marker=marker, s=200, edgecolors='k', label=label, alpha=0.4)
        ax.text(x + 0.015, y + 0.015, obj_id, fontsize=18)
    # plot reference points
    generated_dots = json.load(open(generated_dots_path, "r"))
    reference_points = generated_dots.get("reference_points", {})
    for point_name, pos in reference_points.items():
        ax.scatter(pos[1], pos[0], c='orange', marker='x', s=20, label=point_name)
        ax.text(pos[1] + 0.01, pos[0] + 0.01, point_name, fontsize=16, color='black')
    # Plot trajectories
    for obj_id, positions in interested_pos_history.items():
        if not positions:
            continue
        xs = [pos[1] for pos in positions]
        ys = [pos[0] for pos in positions]
        ax.plot(xs, ys, label=f"{obj_id} trajectory", linewidth=2, color='yellow')
        ax.scatter(xs[-1], ys[-1], c='black', marker='x')  # Mark end point

    # ax.set_xlabel("X")
    # ax.set_ylabel("Y")
    ax.tick_params(axis='both', which='major', labelsize=14)
    ax.set_ylim((0.35, -0.35))
    ax.set_xlim((-0.35, 0.35))
    ax.set_title("Object Trajectories")
    # ax.legend()
    ax.grid(True)
    ax.set_aspect('equal')
    # save plot
    plt.tight_layout()
    save_dir = "/home/s2644572/robosuite/myCode/my_planning_app/logs/plots/phase_1"
    save_path = Path(save_dir) / file_name
    plt.savefig(save_path, dpi=300)
    print(f"Plot saved to {save_path}")
    plt.show()



def _build_object_description_mapping(objects_dict):
    """
    Given a dictionary of objects with properties including 'color' and 'shape',
    return a mapping from object names (e.g., 'obj0') to descriptive strings 
    (e.g., 'green cube').
    """
    return {
        obj_name: f"{props['color']} {props['shape']}"
        for obj_name, props in objects_dict.items()
    }

if __name__ == "__main__":
    # Example usage
    log_path = ["/home/s2644572/robosuite/myCode/my_planning_app/logs/scene_01/experiment_logs/phase_1/basic_stack_the_blue_cube_on_the_green_cube_20250723_165206.json",
                "/home/s2644572/robosuite/myCode/my_planning_app/logs/scene_01/experiment_logs/phase_1/ambiguous_move_the_red_cylinder_closer_to_the_green_cube_20250723_165807.json",
                "/home/s2644572/robosuite/myCode/my_planning_app/logs/scene_01/experiment_logs/phase_1/trajectory_move_the_red_cylinder_to_point_4,_then_to_point_1_20250723_172453.json"]
    interested_objs = [["blue cube"],["red cylinder"],["red cylinder"]]  # Replace with your actual object IDs
    file_names = ["basic_trajectory_plot.png", "ambiguous_trajectory_plot.png", "trajectory_plot.png"]
    generated_dots_path = "/home/s2644572/robosuite/myCode/my_planning_app/logs/scene_01/generated_dots.json"
    for path, interested_objs, file_name in zip(log_path, interested_objs, file_names):
        print(f"Processing log: {path}")
        init_objs, interested_pos_history = trajecotry_extractor(path, interested_objs)
        # print("Initial Objects:", init_objs)
        # print("Interested Position History:", interested_pos_history)
        # Plot the trajectories
        plot_trajectory(init_objs, generated_dots_path,file_name, interested_pos_history)