import json
from typing import List, Dict


def build_prompt(
    command: str,
    task_type: str,
    environment_objects: List[Dict],
    reference_points: Dict[str, List[float]],
    include_examples: bool = True
) -> str:
    """
    Constructs a full prompt for the LLM parser using a user-specified command,
    environment objects, and reference points.
    
    Args:
        command: The natural language instruction.
        environment_objects: A list of dicts representing objects.
        reference_points: A dict of reference point names and [x, y] coords.
        include_examples: Whether to include the predefined prompt examples.

    Returns:
        A formatted prompt string ready for LLM inference.
    """

    # Core environment info
    env_json = {
        "environment": {
            "objects": environment_objects
        },
        "reference_points": reference_points
    }

    # Optional example commands (use the ones from your original prompt)
    examples = """
Example commands:
Command: "Stack the red cube onto the green cylinder"
Output:
{
  "task_type": "simple",
  "action": "stack",
  "interested_object": "red cube",
  "target_reference_object": "green cylinder",
  "success_criteria": [0.0, 0.0, 0.875],
  "explanation": "For the simple task - stack action the evaluation criteria is a specific 3-element vector. The red cube is stacked on top of the green cylinder at its center position. So the x, y coordinates are the same as the cylinder's, and the z coordinate is the height of the cylinder plus the half-height of red cube. So the success criteria is a coordinate, 3-element vector."
}

Command: "Move the blue cylinder to [0.15, 0.0, 0.825]"
Output:
{
  "task_type": "simple",
  "action": "deliver",
  "interested_object": "blue cylinder",
  "success_criteria": [0.15, 0.0, 0.825]
  "explanation": "For the simple task - deliver action, we move it to the specified position in 3D space. So the success criteria is a coordinate, 3-element vector."
}

Command: "Lift the red cube"
Output:
{
  "task_type": "simple",
  "action": "lift",
  "interested_object": "red cube",
  "success_criteria": [-0.2, -0.2, z] # z > 0.875,
  "explanation": "For the simple task - lift action, we raise the red cube vertically so the x,y coordinates remain the same. As long as the z coordinate is greater than the height of the red cube (0.85), it is considered a successful lift. So, the success criteria is a coordinate with z > 0.875."
}

Command: "Put the green cube closer to the blue cube"
Output:
{
  "task_type": "ambiguous",
  "action": "move_closer",
  "interested_object": "green cube", "blue cube",
  "success_criteria": "distance between the green cube and the blue cube at the final state is less than the distance at the start state",
  "explanation": "For the ambiguous task - move_closer action, we need to ensure the objects are closer together than they were at the start. The success criteria is based on the distance between the two objects."   
}

Command: "Separate the blue cylinder and the red cylinder"
output:
{
  "task_type": "ambiguous",
  "action": "move_further",
  "interested_object": "blue cylinder", "red cylinder"
  "success_criteria": "distance between the blue cylinder and the red cylinder at the final state is greater than the distance at the start state",
  "explanation": "For the ambiguous task - move_further action, we need to ensure the final distance between interested objects is greater than the initial distance. The success criteria is based on the distance between the two objects."
}

Command: "Move the green cube to point_5 via point_1"
Ouput:
{
  "task_type": "trajectory",
  "action": "move_via",
  "interested_object": "green cube",
  "interested_waypoints_in_order": ["point_1", "point_5"],
  "success_criteria": [[-0.1,-0.1], [-0.023, 0.107]],",
  "explanation": "For the trajectory task - move_via action, we need to ensure the green cube reaches point_5 after passing through point_1. The success criteria is defined by passing the waypoints in order, so it is a list of coordinates (only x, y coordinates are cared in this case) for the waypoints."
}

Command: "Move the red cylinder to the opposite side of the blue cylinder"
Output:
{
  "task_type": "trajectory",
  "action": "move_opposite",
  "interested_object": "red cylinder",
  "interested_reference_object": "blue cylinder",
  "success_criteria": "[-0.15,-0.15, 0.825]",
  "explanation": "For the trajectory task - move_opposite action,\nStep 1: Find the center position of the blue cylinder and red cylinder, [0.0,0.0,0.825] and [0.15,0.15,0.825] respectively.\nStep 2: Find the opposite position of the red cylinder with respect to the blue cylinder, [0.0,0.0,0.825] + ([0.0,0.0,0.825] - [0.15,0.15,0.825]), which is [-0.15,-0.15, 0.825].\nStep 3: Move the red cylinder to this position."
}
"""

    # Construct the full system prompt
    prompt = f"""You are an instruction parser for a robotic manipulation evaluation system.
Given a natural language command and its tasks type, extract the action and relevant entities and success criteria in a dictionary.

Only the following actions are valid for each task type:
- For **simple** tasks: "lift", "deliver", "stack"
- For **ambiguous** tasks: "move_closer", "move_further"
- For **trajectory** tasks: "move_via", "move_opposite"

Do not infer actions beyond this list. Make sure the action you extract is consistent with the specified task type.

Example Environment:
{json.dumps(env_json, indent=2)}

{examples if include_examples else ""}

Now parse this command and return only the JSON:
Command: "{command}"
Task Type: "{task_type}"
Output:"""

    return prompt
