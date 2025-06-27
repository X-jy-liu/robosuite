import json
from pathlib import Path
from trajectory_planning.json_loader import load_objects_from_json, load_dots_from_json
from trajectory_planning.rpm_generator import RPMGenerator

scene_json_path = Path.home() / "robosuite" / "myCode" / "my_planning_app" / "prompts" / "env_and_func.json"
dots_json_path = Path.home() / "robosuite" / "myCode" /"my_planning_app" / "prompts" / "generated_dots.json"

scene_data = load_objects_from_json(scene_json_path)
dots_data = load_dots_from_json(dots_json_path)

rpm_generator = RPMGenerator(
    objects_pos=scene_data, 
    ref_pnt_pos=dots_data,
    max_rpm=500
)
graph, nodes = rpm_generator.build(bounds=([-0.3, 0.3], [-0.3, 0.3]))

# plot the roadmap
rpm_generator.visualize_prm(graph, nodes, scene_data, dots_data)