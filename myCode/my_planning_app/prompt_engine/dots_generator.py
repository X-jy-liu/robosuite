import random
import json
import math
from models import ObjectSpec
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
        print(f"✅ Loaded {len(self.objects)} objects from: {env_and_func_path}")

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
            x = round(random.uniform(x_min, x_max), 3)
            y = round(random.uniform(y_min, y_max), 3)
            if is_valid_dot(x, y):
                valid_dots.append((x, y))
            attempts += 1

        if len(valid_dots) < num_dots:
            print(f"⚠️ Only found {len(valid_dots)} valid dots after {max_attempts} attempts.")
        return valid_dots

if __name__ == "__main__":
    # Example usage
    from pathlib import Path

    HOME_DIR = Path.home()
    env_path = HOME_DIR / "robosuite" / "myCode" / "my_planning_app" / "prompts" / "env_and_func.json"

    generator = DotGenerator(env_path)
    dots = generator.generate_valid_dots(num_dots=5, clearance=0.08)

    for dot in dots:
        print(f"Dot at position: {dot}")
