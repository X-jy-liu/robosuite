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

    # example environment
    example_env_json = {
    "environment": {
    "objects": [
      { "name": "obj0", "shape": "cube", "color": "red", "position": [-0.2, -0.2], "size": 0.05 },
      { "name": "obj1", "shape": "cube", "color": "blue", "position": [0.2, -0.2], "size": 0.05 },
      { "name": "obj2", "shape": "cube", "color": "green", "position": [-0.2, 0.2], "size": 0.05 },
      { "name": "obj3", "shape": "cylinder", "color": "red", "position": [0.15, 0.15], "size": 0.05 },
      { "name": "obj4", "shape": "cylinder", "color": "blue", "position": [0.0, 0.0], "size": 0.05 }
        ]
      }
    }

    example_ref_pnt_json = {
    "reference_points": {
        "point_1": [
            -0.1, 
            -0.1
        ],
        "point_2": [
            0.1, 
            -0.1
        ],
        "point_3": [
            0.0, 
            -0.2
        ],
        "point_4": [
            0.05, 
            0.2
        ],
        "point_5": [
            -0.29, 
            0.0
        ]
      }
    }


    # example commands
    example_commands = """
Example commands:
Command: "Stack the red cube onto the green cube"
Output:
{
  "task_type": "basic",
  "action": "stack",
  "interested_object": "red cube",
  "target_reference_object": "green cube",
  "success_criteria": [-0.2, 0.2, 0.875],
  "explanation": "For the basic task - stack action the evaluation criteria is a specific 3-element vector. The red cube is stacked on top of the green cube at its center position where the green cube is the target reference object. So the x, y coordinates are the same as the green cube's [-0.2,0.2], and the z coordinate is 0.875 for all stack actions. So the success criteria is a coordinate, 3-element vector."
}

Command: "Stack the red cylinder onto the blue cylinder"
Output:
{
  "task_type": "basic",
  "action": "stack",
  "interested_object": "red cylinder",
  "target_reference_object": "blue cylinder",
  "success_criteria": [0.0, 0.0, 0.875],
  "explanation": "For the basic task - stack action the evaluation criteria is a specific 3-element vector. The red cylinder is stacked on top of the blue cylinder at its center position where the blue cylinder is the target reference object. So the x, y coordinates are the same as the green cylinder's (target reference object), and the z coordinate is 0.875 for all stack actions. So the success criteria is a coordinate, 3-element vector."
}

Command: "Move the blue cylinder to [0.15, 0.0, 0.825]"
Output:
{
  "task_type": "basic",
  "action": "deliver",
  "interested_object": "blue cylinder",
  "success_criteria": [0.15, 0.0, 0.825]
  "explanation": "For the basic task - deliver action, we move it to the specified position in 3D space. So the success criteria is a coordinate, 3-element vector."
}

Command: "Lift the red cube"
Output:
{
  "task_type": "basic",
  "action": "lift",
  "interested_object": "red cube",
  "success_criteria": [-0.2, -0.2, 0.85],
  "explanation": "For the basic task - lift action, we raise the red cube vertically so the x,y coordinates remain the same. For all lift actions, we set the z coordinate of the success criteria to be 0,85, which is the height of the table pluse the object's size. So the success criteria is a coordinate, 3-element vector."
}

Command: "Lift the blue cube"
Output:
{
  "task_type": "basic",
  "action": "lift",
  "interested_object": "blue cube",
  "success_criteria": [0.2, -0.2, 0.85],
  "explanation": "For the basic task - lift action, we raise the red cube vertically so the x,y coordinates remain the same as the blue cube's initial position. For all lift actions, we set the z coordinate of the success criteria to be 0,85, which is the height of the table pluse the object's size. So the success criteria is a coordinate, 3-element vector."
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
  "success_criteria": [[-0.2, 0.2], [-0.1,-0.1], [-0.023, 0.107]],",
  "explanation": "For the trajectory task - move_via action, we need to ensure the green cube reaches point_5 after passing through point_1. The success criteria is defined by passing the waypoints in order, so it is a list of coordinates (only x, y coordinates are cared in this case) for the waypoints."
}

Command: "Move the red cylinder to the opposite side of the blue cylinder"
Output:
{
  "task_type": "trajectory",
  "action": "move_opposite",
  "interested_object": "red cylinder",
  "success_criteria": "[0.15, 0.15],[-0.15,-0.15]",
  "explanation": "For the trajectory task - move_opposite action,\nStep 1: Find the center position of the blue cylinder and red cylinder, [0.0,0.0,0.825] and [0.15,0.15,0.825] respectively.\nStep 2: Find the opposite position of the red cylinder with respect to the blue cylinder, [0.0,0.0,0.825] + ([0.0,0.0,0.825] - [0.15,0.15,0.825]), which is [-0.15,-0.15, 0.825].\nStep 3: Move the red cylinder to this position."
}
"""

    # Construct the full system prompt
    prompt = f"""You are an instruction parser for a robotic manipulation evaluation system.
Given a natural language command and its tasks type, extract the action and relevant entities and success criteria in a dictionary.

Only the following actions are valid for each task type:
- For **basic** tasks: "lift", "deliver", "stack"
- For **ambiguous** tasks: "move_closer", "move_further"
- For **trajectory** tasks: "move_via", "move_opposite"

Each "interested_object" must always be written as two words:
- The **first word** must be a valid color (e.g., red, green, blue).
- The **second word** must be a valid shape (e.g., cube, cylinder).
For **ambiguous** tasks, you must return exactly two interested objects, separated by commas.
Never return the object with other information like, position, size, etc.

For the success criteria of trajectory tasks, you must return list(list(float, floart)). And the first element of the list must be the x-y position of the object at the initial state.

Do not infer actions beyond this list. Make sure the action you extract is consistent with the specified task type.

Example Environment:
{example_env_json}

Example Reference Points:
{example_ref_pnt_json}

{example_commands if include_examples else ""}

Now parse this command and return only the JSON:
Command: "{command}"
Task Type: "{task_type}"
Current Environment: {json.dumps(env_json, indent=2)}
The output should be a dictionary with the following keys:
- "task_type": The type of task (one of "basic", "ambiguous", "trajectory").
- "action": The action to be evaluated, following the valid actions for the task type.
- "interested_object": The object(s) of interest for the evaluation.
- "target_reference_object": The reference object for the action, only applicable to for basic task - stack action.
- "success_criteria": Criteria for success evaluation which can vary based on task type.
- "explanation": A brief explanation of how the success criteria is determined.

for the basic task type stack action: the z-coordinate must be 0.875, and it needs to output "target_reference_object" as the object that the interested object is stacked on top of.
for the basic task type deliver action: the z-coordinate must be 0.825
for the basic task type lift action: the z-coordinate must be 0.85

Output:"""

    return prompt
