import random
import json
import math
from .models import ObjectSpec
from typing import List, Tuple, Optional
from pathlib import Path

class DotGenerator:
    def __init__(
        self,
        env_and_func_path: Path,
        table_bounds: Tuple[Tuple[float, float], Tuple[float, float]] = ((-0.35, 0.35), (-0.35, 0.35))
    ):
        self.env_and_func_path = env_and_func_path
        self.table_bounds = table_bounds
        self.objects: List[ObjectSpec] = self._load_objects()
        print(f"Loaded {len(self.objects)} objects from: {env_and_func_path}")

    def _load_objects(self) -> List[ObjectSpec]:
        with open(self.env_and_func_path, "r") as f:
            env_data = json.load(f)
        return [ObjectSpec(**obj) for obj in env_data["environment"]["objects"]]

    def generate_valid_dots(
        self,
        num_dots: int = 5,
        clearance: float = 0.08,
        seed: Optional[int] = None
    ) -> List[Tuple[float, float]]:
        """
        Generate random dots on the table that are at least `clearance` away from all objects.
        """
        if seed is not None:
            random.seed(seed)

        valid_dots = []
        x_min, x_max = self.table_bounds[0]
        y_min, y_max = self.table_bounds[1]

        def is_valid_dot(x: float, y: float) -> bool:
            for obj in self.objects:
                ox, oy = obj.position[:2]
                distance = math.sqrt((x - ox) ** 2 + (y - oy) ** 2)
                if distance < (obj.size + clearance):
                    return False
            return True

        attempts = 0
        max_attempts = 500
        while len(valid_dots) < num_dots and attempts < max_attempts:
            # Soft-bias sampling: pick a random object as center
            obj = random.choice(self.objects)
            ox, oy = obj.position[:2]

            # Sample from a Gaussian centered at the object
            x = round(random.gauss(ox, 0.08), 3)  # std deviation controls spread
            y = round(random.gauss(oy, 0.08), 3)

            # Clamp to table bounds
            if not (x_min <= x <= x_max and y_min <= y <= y_max):
                attempts += 1
                continue

            if is_valid_dot(x, y):
                valid_dots.append((x, y))
            attempts += 1


        if len(valid_dots) < num_dots:
            print(f"⚠️ Only found {len(valid_dots)} valid dots after {max_attempts} attempts.")
        return valid_dots
    
    def save_dots_to_json(self, dots: List[Tuple[float, float]], output_path: Path):
        """
        Save the generated dots to a JSON file in the format:
        {
            "reference_points": {
                "point_1": [x1, y1],
                "point_2": [x2, y2],
                ...
            }
        }
        """
        reference_points = {
            f"point_{i + 1}": [x, y] for i, (x, y) in enumerate(dots)
        }

        with open(output_path, "w") as f:
            json.dump({"reference_points": reference_points}, f, indent=4)

        print(f"✅ Saved {len(dots)} dots to: {output_path}")

    def load_dots_from_json(self, input_path: Path) -> List[Tuple[float, float]]:
        """
        Load dots from a JSON file saved using `save_dots_to_json`, and return them as a list of (x, y) tuples.
        """
        with open(input_path, "r") as f:
            data = json.load(f)

        reference_points = data.get("reference_points", {})
        dots = [tuple(coord) for coord in reference_points.values()]
        print(f"✅ Loaded {len(dots)} dots from: {input_path}")
        return dots



if __name__ == "__main__":
    # Example usage (need to remove the . before models when importing it)
    from pathlib import Path

    HOME_DIR = Path.home()
    env_path = HOME_DIR / "robosuite" / "myCode" / "my_planning_app" / "prompts" / "scene_config_example.json"

    generator = DotGenerator(env_path)
    dots = generator.generate_valid_dots(num_dots=5, clearance=0.08)
    # save the dots to a JSON file
    output_path = HOME_DIR / "robosuite" / "myCode" / "my_planning_app" / "prompts" / "generated_dots.json"
    # generator.save_dots_to_json(dots, output_path)
    print(generator.load_dots_from_json(output_path))

    for dot in dots:
        print(f"Dot at position: {dot}")
