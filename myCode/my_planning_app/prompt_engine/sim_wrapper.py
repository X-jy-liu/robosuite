# prompt_engine/sim_wrapper.py
from myCode.skill_executor import SkillExecutor
from myCode.config_controller import controller
from myCode.my_env.multi_object_lift import MultiObjectLift
from myCode.config_controller import controller
from myCode.my_planning_app.prompt_engine.prompt_loader import load_default_prompt
from myCode.my_planning_app.prompt_engine.models import ObjectSpec
import matplotlib.pyplot as plt
from pathlib import Path
from ultralytics import YOLO
import json
from typing import List, Dict, Tuple

class SimWrapper:
    def __init__(self, scene_config_path=None, has_render=False, record_video=False, video_path="execution_video.mp4", robot_offset=False):
        # Initialize the environment with the controller configuration
        self.scene_config_path = scene_config_path
        self.has_render = has_render  # whether to render the scene
        self.robot_offset = robot_offset  # keep robot at the default position
        self.record_video = record_video  # whether to record video
        self.video_path = video_path  # video path
        self.env = MultiObjectLift(
            robots="Panda",
            controller_configs=controller,
            has_renderer=self.has_render,
            scene_config_path=self.scene_config_path,
            ignore_done=True,
            robot_offset=self.robot_offset,
            render_visual_mesh=True,
            has_offscreen_renderer=True,
        )
        self.env.reset()

        for body_name in self.env.sim.model.body_names:
            if "cube_main" in body_name:
                raise ValueError(f"The default cube '{body_name}' still exists in the environment.")
        self.executor = SkillExecutor(self.env, record_video=self.record_video, video_path=self.video_path)

    def get_current_state(self):
        return self.executor.get_all_object_descriptions()

    def execute_plan(self, plan):
        """
        1. Execute a symbolic plan in the MuJoCo simulation.
        2. Returns the history of object positions after execution.
        """

        print("\nLLM generated plan:\n", plan)
        obj_specs_history = self.executor.execute_plan(plan)

        return obj_specs_history
    
    def scene_render(self, save_dir:Path):
        """
        Renders the scene json into a RGB top down view image.
        Inputs:
            save_dir (Path): directory to save the rendered image.
        """
        save_dir.mkdir(parents=True, exist_ok=True)
        
        # Load the scene
        self.env.reset()  # Make sure this reloads from `scene_config_path`

        self.env._setup_camera()
        self.env.sim.forward()

        # Render RGB image
        image = self.env.sim.render(camera_name="birdview", width=512, height=512)
        print(f"Rendered image shape: {image.shape}")
        
        # Use filename prefix from scene_path
        base_name = self.scene_config_path.stem + "_rendered"
        image_path = save_dir / f"{base_name}.png"
        
        # Save image
        plt.imsave(image_path, image)
        print(f"Scene rendered and saved to {image_path}")
    
    def yolo_perception_and_save(self, gt_env_and_func_path: Path, rendered_img_path: Path, json_output_name: str, checkpoint_path: Path):
        """
        Perform YOLO perception on the rendered image and update the GT scene JSON.

        Args:
            gt_env_and_func_path (Path): path to the original GT env_and_func.json file.
            rendered_img_path (Path): path to the corresponding top-down image.
            checkpoint_path (Path): path to the YOLOv8 model checkpoint (.pt).
        """
        table_size_m = 0.8
        image_size_px = 512
        
        try:
            # === Load model ===
            model = YOLO(str(checkpoint_path))
            print(f"Loaded YOLO model from {checkpoint_path}")

            # === Run inference on the rendered image ===
            results = model(str(rendered_img_path),conf=0.8)
            detections = results[0].to_json()
            parsed = json.loads(detections)

            # === Convert YOLO detections to new object specs ===
            meters_per_pixel = table_size_m / image_size_px  # 0.8 / 512
            new_objects = []
            for i, det in enumerate(parsed):
                box = det["box"]
                center_x = (box['x1'] + box['x2']) / 2
                center_y = (box['y1'] + box['y2']) / 2

                # Convert pixel to sim coordinates (x: left→right, y: top→bottom flipped)
                sim_x = round((center_x - 256) * meters_per_pixel, 3)
                sim_y = -round((256 - center_y) * meters_per_pixel, 3)

                color, shape = det["name"].split("_", 1)
                obj = {
                    "name": f"obj{i}",
                    "shape": shape,
                    "color": color,
                    "position": [sim_x, sim_y],
                    "size": 0.05
                }
                new_objects.append(obj)

            print(f"Detected {len(new_objects)} objects. Replacing environment.objects in GT JSON.")

            # === Load GT JSON and replace the objects list ===
            with open(gt_env_and_func_path, "r") as f:
                gt_data = json.load(f)

            gt_objects = gt_data["environment"]["objects"]

            new_objects = self._match_yolo_to_gt_objects(gt_objects, new_objects)

            output_path = gt_env_and_func_path.with_name(gt_env_and_func_path.stem + json_output_name)
            self._save_pred_json(new_objects, output_path)

        except Exception as e:
            print(f"Error during YOLO perception: {e}")

    def _save_pred_json(self, new_objects: List[Dict], output_path: Path):
        """
        Save predicted objects into a formatted JSON file for testing, preserving structure.

        Args:
            output_path (Path): Where to save the JSON file.
            new_objects (List[Dict]): List of predicted and matched objects.
        """
        # Format each object as a compact one-line JSON string
        formatted_objects = [
            f'{{ "name": "{obj["name"]}", "shape": "{obj["shape"]}", "color": "{obj["color"]}", '
            f'"position": {json.dumps([round(c, 3) for c in obj["position"]])}, "size": {obj["size"]} }}'
            for obj in new_objects
        ]
        objects_str = ",\n      ".join(formatted_objects)

        shape_details_str = (
            '  "shape_details": {\n'
            '    "cube": {\n'
            '      "size": [0.05, 0.05, 0.05],\n'
            '      "description": "Each cube has equal dimensions of 0.05 meters."\n'
            '    },\n'
            '    "cylinder": {\n'
            '      "height": 0.05,\n'
            '      "diameter": 0.05,\n'
            '      "description": "Each cylinder is 0.05 meters tall and 0.05 meters in diameter."\n'
            '    }\n'
            '  },\n'
        )

        # Wrap full JSON
        json_str = (
            '{\n'
            '  "environment": {\n'
            '    "objects": [\n'
            f'      {objects_str}\n'
            '    ]\n'
            '  },\n'
            f'{shape_details_str}'
            '  "available_functions": {\n'
            '    "move": {\n'
            '      "params": ["target"],\n'
            '      "description": "Target can be one of \'obj0\', \'obj1\', \'obj2\', \'obj3\', or \'obj4\', or a specific 3D Cartesian coordinate in the form [x, y, z], such as [0.1, 0.2, 0.8], representing the position in meters.",\n'
            '      "examples": [["move", "obj1"], ["move", [0.1, 0.1, 0.835]]]\n'
            '    },\n'
            '    "grip_and_pickup": {\n'
            '      "params": ["object"],\n'
            '      "description": "It can only follow the \'move\' function. The object is one of \'obj0\', \'obj1\', \'obj2\', \'obj3\', \'obj4\'. It grabs it to the above position",\n'
            '      "examples": [["grip_and_pickup", "obj2"]]\n'
            '    },\n'
            '    "gripper_close": {\n'
            '      "params": [],\n'
            '      "description": "Close the gripper.",\n'
            '      "examples": [["gripper_close"]]\n'
            '    },\n'
            '    "gripper_open": {\n'
            '      "params": [],\n'
            '      "description": "Open the gripper.",\n'
            '      "examples": [["gripper_open"]]\n'
            '    }\n'
            '  }\n'
            '}'
        )

        # Write to file
        with open(output_path, "w") as f:
            f.write(json_str)
        print(f"Saved updated env_and_func JSON to {output_path}")

    def _match_yolo_to_gt_objects(
            self,
            gt_objects: List[Dict],
            pred_objects: List[Dict]
        ) -> List[Dict]:
        """
        Match YOLO-predicted objects to GT objects using greedy nearest-neighbor
        strategy based on shape+color type and spatial distance.
        Only returns objects that were actually detected.
        """
        matched_objects = []
        used_indices = set()

        for gt in gt_objects:
            gt_type = (gt["color"], gt["shape"])
            gt_pos = gt["position"]

            best_idx = None
            best_dist = float("inf")

            for i, pred in enumerate(pred_objects):
                if i in used_indices:
                    continue
                pred_type = (pred["color"], pred["shape"])
                if pred_type != gt_type:
                    continue
                # Compute Euclidean distance
                dist = ((pred["position"][0] - gt_pos[0]) ** 2 + 
                        (pred["position"][1] - gt_pos[1]) ** 2) ** 0.5
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i

            if best_idx is not None:  # Only add if a match was found
                used_indices.add(best_idx)
                matched_obj = pred_objects[best_idx].copy()
                matched_obj["name"] = gt["name"]  # Preserve GT naming
                matched_objects.append(matched_obj)
            # Remove the else clause - don't add undetected objects

        return matched_objects


    # def set_object_pose(self, object_name, position, orientation = None):
    #     """
    #     Sets the pose of an object in the MuJoCo simulation by name.
    #     Args:
    #         object_name (str): name of the object (must match mujoco body name).
    #         position (list): [x, y, z]
    #         orientation (list): [x, y, z, w] quaternion
    #     """
    #     sim = self.env.sim

    #     if orientation is None:
    #         orientation = [0, 0, 0, 1]

    #     try:
    #         body_id = sim.model.body_name2id(object_name)
    #         sim.model.body_pos[body_id] = position
    #         sim.model.body_quat[body_id] = orientation
    #         sim.forward()
    #         print(f"✅ Object '{object_name}' pose set to {position}, {orientation}")
    #     except Exception as e:
    #         print(f"❌ Failed to set pose for '{object_name}': {e}")


    # def reset_scene(self):
    #     """
    #     Reload the initial scene setup without restarting the viewer or creating a new instance.
    #     """
    #     try:
    #         # Load initial scene from config again
    #         base_prompt = load_default_prompt(self.scene_config_path)
    #         raw_objects = base_prompt.environment.get("objects", [])
    #         objects = [ObjectSpec(**obj) for obj in raw_objects.values()]
    #         print("debugging ...")
    #         print(f"type of objects: {type(objects)}")
    #         print(f"objects: {objects}")
    #         # Reset the environment manually
    #         self.env.reset()

    #         # Reset objects to initial positions
    #         for obj in objects:
    #             name = obj["name"]
    #             pos = obj["position"]
    #             self.set_object_pose(name, pos)
    #     except Exception as e:
    #         print(f"Error resetting scene: {e}")

    #     # Reset robot (if needed)
    #     self.reset_robot()

    #     print("🔄 SimWrapper scene reset from saved config.")

    # def reset_robot(self):
    #     self.executor.reset_robot()

    # def idle_and_close(self):
    #     time.sleep(1)
    #     self.env.close()
    #     self.env = None