import sys
import os
import json
import numpy as np
import matplotlib.pyplot as plt
from robosuite.models.objects.primitive.box import BoxObject
from robosuite.models.objects.primitive.cylinder import CylinderObject
from robosuite.utils.mjcf_utils import add_to_dict
from robosuite.environments.manipulation.lift import Lift
# Add parent directory to sys.path
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)
from config_controller import controller


# Set up directory
os.makedirs("scene_output", exist_ok=True)

# Object configuration
SHAPES = ["cube", "cylinder"]
COLORS = {
    "red": [1, 0, 0, 1],
    "green": [0, 1, 0, 1],
    "blue": [0, 0, 1, 1]
}

# load OSC controller config
osc_config = controller
# Custom environment
class MultiObjectLift(Lift):
    def _load_model(self):
        super()._load_model()
        table_z = self.table_offset[2]
        self.robots[0].robot_model.set_base_xpos([-2, 0, 0])
        self.object_metadata = []
        placed_positions = []
        min_dist = 0.07
        
        for i in range(5):
            shape = np.random.choice(SHAPES)
            color_name = np.random.choice(list(COLORS))
            color_rgba = COLORS[color_name]

            obj = BoxObject(name=f"obj{i}", size=[0.025]*3, rgba=color_rgba) if shape == "cube" else \
                  CylinderObject(name=f"obj{i}", size=[0.025, 0.025], rgba=color_rgba)

            for _ in range(100):
                pos_xy = np.random.uniform(-0.35, 0.35, 2)
                if all(np.linalg.norm(pos_xy - np.array(p[:2])) > min_dist for p in placed_positions):
                    break
            pos = [pos_xy[0], pos_xy[1], table_z + 0.0125]
            placed_positions.append(pos)

            obj.get_obj().set("pos", f"{pos[0]} {pos[1]} {pos[2]}")
            self.model.merge_objects([obj])
            add_to_dict(self.model.worldbody, "body", obj.get_obj())
            self.object_metadata.append({"name": f"obj{i}", "shape": shape, "color": color_name, "position": pos})

    def _setup_camera(self):
        cam_id = self.sim.model.camera_name2id("birdview")
        self.sim.model.cam_pos[cam_id] = [0.0, 0.0, 1.5]
        self.sim.model.cam_quat[cam_id] = [1.0, 0.0, 0.0, 0.0]
        self.sim.model.cam_fovy[cam_id] = 60

# Generate one scene
env = MultiObjectLift(
    robots="Panda",
    controller_configs=osc_config,
    has_renderer=True,
    camera_names="birdview",
    camera_heights=512,
    camera_widths=512,
    camera_depths=False
)

env.reset()
env._setup_camera()
env.sim.forward()

# Remove default cube
for body_name in env.sim.model.body_names:
    if "cube_main" in body_name:
        addr = env.sim.model.get_joint_qpos_addr("cube_joint0")
        env.sim.data.qpos[addr[0]:addr[0]+7] = [5.0, 5.0, 0.0, 1, 0, 0, 0]
        env.sim.forward()

# Save image and metadata
image = env.sim.render(camera_name="birdview", width=512, height=512)
plt.imsave("scene_output/scene_1.png", image)

with open("scene_output/scene_1.json", "w") as f:
    json.dump(env.object_metadata, f, indent=2)

print("Scene saved with 5 objects.")
env.close()
