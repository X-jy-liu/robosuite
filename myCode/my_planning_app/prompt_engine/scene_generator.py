import random
import json
import numpy as np
from typing import List, Tuple, Optional
from pathlib import Path

# Constants
TABLE_Z = 0.8
HALF_HEIGHT = 0.025
CYLINDER_CALIBRATION = 0.0
OBJECT_SIZE = 0.025 * 2

SHAPES = ["cube", "cylinder"]
COLORS = ["red", "green", "blue"]

class SceneGenerator:
    def __init__(
        self,
        table_bounds: Tuple[Tuple[float, float], Tuple[float, float]] = ((-0.25, 0.25), (-0.25, 0.25)),
        min_distance: float = 0.08,
        object_size: float = OBJECT_SIZE
    ):
        self.table_bounds = table_bounds
        self.min_distance = min_distance
        self.object_size = object_size

    def _is_valid_position(self, pos: Tuple[float, float], used_positions: List[Tuple[float, float]]) -> bool:
        for p in used_positions:
            if np.linalg.norm(np.array(p) - np.array(pos)) < self.min_distance:
                return False
        return True

    def generate_scene(self, num_objects: int = 5, seed: Optional[int] = None) -> List[Tuple[str, str, str, List[float]]]:
        if seed is not None:
            random.seed(seed)

        scene = []
        used_positions = []
        attempts = 0
        max_attempts = 100

        x_min, x_max = self.table_bounds[0]
        y_min, y_max = self.table_bounds[1]

        while len(scene) < num_objects and attempts < max_attempts:
            shape = random.choice(SHAPES)
            color = random.choice(COLORS)
            x, y = round(random.uniform(x_min, x_max), 3), round(random.uniform(y_min, y_max), 3)

            if not self._is_valid_position((x, y), used_positions):
                attempts += 1
                continue

            z = TABLE_Z + HALF_HEIGHT
            if shape == "cylinder":
                z += CYLINDER_CALIBRATION

            name = f"obj{len(scene)}"
            scene.append((name, shape, color, [x, y, z]))
            used_positions.append((x, y))
            attempts = 0  # reset attempts after success

        if len(scene) < num_objects:
            print(f"⚠️ Only generated {len(scene)} objects out of {num_objects} after {max_attempts} attempts.")

        return scene

    def save_to_json(self, scene: List[Tuple[str, str, str, List[float]]], save_path: Path):
        # Format each object as a one-liner
        formatted_objects = [
            f'{{ "name": "{name}", "shape": "{shape}", "color": "{color}", '
            f'"position": {json.dumps([round(c, 3) for c in position[:2]])}, '
            f'"size": {self.object_size} }}'
            for name, shape, color, position in scene
        ]

        # Join object lines
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


        # Wrap the full JSON content as string
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

        # Save to file
        with open(save_path, "w") as f:
            f.write(json_str)

        print(f"✅ Scene saved with compact one-line-per-object format to {save_path}")




if __name__ == "__main__":
    generator = SceneGenerator()
    scene = generator.generate_scene(num_objects=5, seed=42)
    HOME_DIR = Path.home()
    save_path = HOME_DIR / 'robosuite' / 'myCode' / 'my_planning_app' / 'prompts' / 'env_and_func.json'
    save_path = Path(save_path).resolve()
    print({f" the scene_path is {save_path}"})
    generator.save_to_json(scene, save_path)