# prompt_engine/prompt_loader.py
import json
from pathlib import Path
from .models import PromptInput

def load_default_prompt(path: str | Path = "prompts/env_and_func.json") -> PromptInput:
    with open(path, "r") as f:
        data = json.load(f)
    return PromptInput(**data)

def load_reference_dots(path: str | Path = "prompts/generated_dots.json") -> dict:
    """
    Load reference points from a JSON file.

    The expected format is:
    {
        "reference_points": {
            "point_1": [x1, y1],
            "point_2": [x2, y2],
            ...
        }
    }

    Returns:
        A dictionary mapping point names (e.g., "point_1") to (x, y) tuples.
    """
    path = Path(path)
    with open(path, "r") as f:
        data = json.load(f)

    reference_points = data.get("reference_points", {})
    return reference_points
