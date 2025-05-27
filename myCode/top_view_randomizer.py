import os
import cv2  # Optional, you can also use plt.imsave()
import robosuite as suite
from robosuite import load_composite_controller_config
from robosuite.models.objects.primitive.box import BoxObject
from robosuite.models.objects.primitive.cylinder import CylinderObject
from robosuite.utils.mjcf_utils import add_to_dict
from robosuite.environments.manipulation.lift import Lift
import matplotlib.pyplot as plt
import numpy as np
import json
from tqdm import tqdm

# osc_config = load_composite_controller_config(controller='BASIC')

osc_config = {
    'type': 'BASIC',
    'body_parts': {
        'right': {
            'type': 'OSC_POSE',
            'input_max': 1,
            'input_min': -1,
            'output_max': [0.05, 0.05, 0.05, 0.5, 0.5, 0.5],
            'output_min': [-0.05, -0.05, -0.05, -0.5, -0.5, -0.5],
            'kp': 150, 'damping_ratio': 1,
            'impedance_mode': 'fixed',
            'kp_limits': [0, 300],
            'damping_ratio_limits': [0, 10],
            'position_limits': None,
            'orientation_limits': None,
            'uncouple_pos_ori': True,
            'input_type': 'delta',
            'input_ref_frame': 'base',
            'interpolation': None,
            'ramp_ratio': 0.2,
            'gripper': {'type': 'GRIP'}
        }
    }
}

# Directory to save images
os.makedirs("randomized_images", exist_ok=True)

# Object pool settings
SHAPES = ["cube", "cylinder"]
COLORS = {
    "red": [1, 0, 0, 1],
    "green": [0, 1, 0, 1],
    "blue": [0, 0, 1, 1]
}

# Custom environment with 5 randomized objects
class MultiObjectLift(Lift):
    def _load_model(self):
        super()._load_model()
        table_z = self.table_offset[2]
        self.robots[0].robot_model.set_base_xpos([-2, 0, 0])
        self.object_metadata = []  # Store object info for later access
        placed_positions = [] # Store the positions of placed objects
        min_dist = 0.07  # Minimum allowed distance between object centers
        
        # Create and add 5 randomized objects
        for i in range(5):
            shape = np.random.choice(SHAPES)
            color_name = np.random.choice(list(COLORS.keys()))
            color_rgba = COLORS[color_name]

            # Choose shape
            if shape == "cube":
                obj = BoxObject(
                    name=f"obj{i}",
                    size=[0.025, 0.025, 0.025],
                    rgba=color_rgba,
                    material=None,
                    obj_type="all"
                )
            else:
                obj = CylinderObject(
                    name=f"obj{i}",
                    size=[0.025, 0.025],  # (radius, height)
                    rgba=color_rgba,
                    material=None,
                    obj_type="all"
                )

            # Random position in X-Y plane
            # Try up to 100 times to find a non-overlapping position
            for _ in range(100):
                pos_xy = np.random.uniform(low=-0.35, high=0.35, size=2)
                if all(np.linalg.norm(pos_xy - np.array(p[:2])) > min_dist for p in placed_positions):
                    break
            else:
                print(f"Warning: Couldn't place object {i} without overlap.")

            pos = [pos_xy[0], pos_xy[1], table_z + 0.0125] # 0.0125 = object half-height
            placed_positions.append(pos)

            obj.get_obj().set("pos", f"{pos[0]} {pos[1]} {pos[2]}")

            # Add to the model
            self.model.merge_objects([obj])
            add_to_dict(self.model.worldbody, "body", obj.get_obj())

            # save the object metadata
            self.object_metadata.append({
                "name": f"obj{i}",
                "shape": shape,
                "color": color_name,
                "position": pos
            })

    def _setup_camera(self):
        cam_id = self.sim.model.camera_name2id("birdview")
        self.sim.model.cam_pos[cam_id] = [0.0, 0.0, 1.5]  # Adjust height as needed
        self.sim.model.cam_quat[cam_id] = [1.0, 0.0, 0.0, 0.0]  # Look straight down
        self.sim.model.cam_fovy[cam_id] = 60  # Smaller FOV to zoom in tighter

# Image generation loop
# num_samples = 10000 # Number of images to generate

# for i in tqdm(range(num_samples),desc="Generating scenes",unit="scene"):
i = 0


while True:
    try:
        env = MultiObjectLift(
            robots="Panda",
            controller_configs=osc_config,
            has_renderer=True,
            camera_names="birdview",
            camera_heights=512,
            camera_widths=512,
            camera_depths=False
        )
        # reset the environment
        env.reset()
        env._setup_camera()
        env.sim.forward()
        # TODO: remove the body with body_names == cube_main out of the table by changing its position (done)
        for body_name in env.sim.model.body_names:
            if "cube_main" in body_name:
                qpos_addr = env.sim.model.get_joint_qpos_addr("cube_joint0")
                env.sim.data.qpos[qpos_addr[0]:qpos_addr[0]+7] = np.array([5.0, 5.0, 0.0, 1, 0, 0, 0])  # pos + unit quaternion
                env.sim.forward()

        image = env.sim.render(camera_name="birdview", width=512, height=512)
        plt.imsave(f"randomized_images/scene_{i+1}.png", image)

        # Save metadata as JSON
        with open(f"randomized_images/scene_{i+1}.json", "w") as f:
            json.dump(env.object_metadata, f, indent=2)
        # close the environment to clear the cache
        env.close()

        print(f"Saved scene {i+1} with {len(env.object_metadata)} objects.")
        i += 1 # Increment the scene counter

    except Exception as e:
        print(f"Error at scene {i+1}: {e}")
        break

    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting gracefully.")
        break

print(f"{i+1} randomized object scenes saved.")