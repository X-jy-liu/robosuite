# prompt_engine/prompt_loader.py
import json
from pathlib import Path
from .models import PromptInput

def load_default_prompt(path: str | Path = "prompts/env_and_func.json") -> PromptInput:
    with open(path, "r") as f:
        data = json.load(f)
    return PromptInput(**data)
