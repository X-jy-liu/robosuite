# prompt_engine/models.py
from pydantic import BaseModel, Field
from typing import List, Dict, Any, Union, Optional


class FunctionSpec(BaseModel):
    params: List[str]
    description: str
    examples: List[List[Any]]


class ObjectSpec(BaseModel):
    """
    Describes an object in the scene.
    Example:
    {
        "name": "red_cube",
        "shape": "cube",
        "color": "red",
        "position": [0.1, 0.2, 0.8125],
        "size": 0.05
    }
    """
    name: str
    shape: str
    color: str
    position: List[float]  # Typically [x, y, z]
    size: float


class PromptInput(BaseModel):
    """
    High-level prompt structure including scene, available robot functions,
    optional examples, and a user instruction.
    """
    environment: Dict[str, List[ObjectSpec]]  # usually just {"objects": [...]}
    available_functions: Dict[str, FunctionSpec]
    examples: List[dict] = []
    instructions: str = ""

class ChatRequest(BaseModel):
    commands: Union[str, List[str]]  # Allow single or multiple commands
    task_type: str
    mode: Optional[str] = None

class InitSessionRequest(BaseModel):
    regenerate_scene: bool = Field(False, description="Whether to regenerate the scene configuration",
                            example=False)
    regenerate_dots: bool = Field(False, description="Whether to regenerate the dots in the scene",
                            example=False)
    ambiguous_effects: bool = Field(False, description="whether to conduct ambigous experiments containing only 3 objects to exmanine the effectiveness of the robot's actions",
                                    example=False)
    scene_number: int = Field(..., description="The scene number to initialize", example=1)
    phase: int = Field(1, description="The phase of the experiment to be chosen from 1, 2, 3. 1 for fully described env, 2 for perception based env, 3 for reality env", example=1)
    has_render: bool = Field(False, description="Whether to render the scene", example=False)
    record_video: bool = Field(True, description="Whether to record the execution video", example=True)
    