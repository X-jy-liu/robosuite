# prompt_engine/prompt_builder.py
from typing import Optional
from .models import PromptInput
from .utils import format_examples

def construct_prompt(payload: PromptInput, command: str, mode: str = "chain", initial_command: Optional[str] = None) -> str:
    obj_descriptions = "\n".join([
        f"- {obj.name}: shape={obj.shape}, color={obj.color}, position={obj.position}, size={obj.size}"
        for obj in payload.environment.get("objects", [])
    ])

    func_descriptions = "\n".join([
        f"- {name}({', '.join(spec.params)}) -> modes: {spec.modes}"
        for name, spec in payload.available_functions.items()
    ])

    example_blocks = format_examples(payload.examples)
    task_instruction = f"""Now, generate a symbolic plan for the original task, 
                            modified by the following instruction:
                            Original Task: {initial_command}
                            Follow-up Instruction: {command}""" if mode == "chain" and initial_command else \
                       f"Now, generate a symbolic plan for this task:\n{command}"

    prompt = f"""
    You are a robot planning assistant...
    ...
    {obj_descriptions}
    ...
    {func_descriptions}
    ...
    {payload.instructions.strip()}
    ...
    {example_blocks}
    ...
    {task_instruction}
    ...
    """.strip()

    return prompt
