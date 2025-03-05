import numpy as np
import time
from robosuite import make
from robosuite.controllers import controller_factory, load_part_controller_config
import openai
import json
import os

import json

def parse_command_with_llm(command):
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": "You are a robot control assistant."},
            {"role": "user", "content": f"Convert this command into structured JSON: {command}"}
        ]
    )

    structured_command = response["choices"][0]["message"]["content"]

    # Convert string response to JSON safely
    try:
        return json.loads(structured_command)
    except json.JSONDecodeError:
        print(f"Error decoding JSON: {structured_command}")
        return None  # Handle the error gracefully


class RoboSkillSet:
    def __init__(self):
        
        # Define the controller (EXPLICITLY as in move_ee.py)
        self.controller = {
            'type': 'BASIC',
            'body_parts': {
                'right': {
                    'type': 'OSC_POSE',
                    'input_max': 1,
                    'input_min': -1,
                    'output_max': [0.05, 0.05, 0.05, 0.5, 0.5, 0.5],
                    'output_min': [-0.05, -0.05, -0.05, -0.5, -0.5, -0.5],
                    'kp': 150, 'damping_ratio': 1,
                    'impedance_mode': 'fixed',
                    'kp_limits': [0, 300],
                    'damping_ratio_limits': [0, 10],
                    'position_limits': None,
                    'orientation_limits': None,
                    'uncouple_pos_ori': True,
                    'input_type': 'delta',
                    'input_ref_frame': 'base',
                    'interpolation': None,
                    'ramp_ratio': 0.2,
                    'gripper': {'type': 'GRIP'}
                }
            }
        }
        self.env = make(env_name="Lift", robots="Panda", controller_configs=self.controller, has_renderer=True)
        self.obs = self.env.reset()

    def move_ee(self, target_pos):
        """ Moves the robot arm to the target position. """
        for _ in range(500):
            current_pos = self.obs["robot0_eef_pos"]
            error = target_pos - current_pos
            action = np.concatenate([error * 3, [0, 0, 0], [0]])
            self.obs, _, _, _ = self.env.step(action)
            self.env.render()

            if np.linalg.norm(error) < 0.01:
                print(f"Reached position {target_pos}")
                return True
        return False

    def grip(self, state):
        """ Controls the gripper: -1 (open), 1 (close) """
        action = np.array([0, 0, 0, 0, 0, 0, state])
        for _ in range(10):
            self.env.step(action)
            self.env.render()

    def execute_task(self, command):
        """ Uses LLM to process commands and execute corresponding actions. """
        structured_command = parse_command_with_llm(command)

        if structured_command["action"] == "move":
            target_pos = np.array(structured_command["target"])
            self.move_ee(target_pos)

        elif structured_command["action"] == "pick":
            if structured_command["target"] == "cube":
                cube_pos = self.obs.get("cube_pos", None)
                if cube_pos is None:
                    raise ValueError("Cube position not found!")
                self.move_ee(cube_pos + np.array([0, 0, 0.1]))  # Move above the cube
                self.grip(-1)  # Open gripper
                self.move_ee(cube_pos)  # Move to cube
                self.grip(1)  # Close gripper
                self.move_ee(cube_pos + np.array([0, 0, 0.1]))  # Lift up

        elif structured_command["action"] == "drop":
            target_drop_pos = np.array(structured_command["target"])
            self.move_ee(target_drop_pos + np.array([0, 0, 0.1]))  # Hover above target
            self.move_ee(target_drop_pos)  # Move to target
            self.grip(-1)  # Open gripper to drop

if __name__ == "__main__":
    openai.api_key = os.getenv("OPENAI_API_KEY")
    if openai.api_key is None:
        raise ValueError("OpenAI API Key is not set! Please export OPENAI_API_KEY.")

    robot = RoboSkillSet()

    # Step 1: Move Above the Object
    cube_pos = np.array([-0.02196927, -0.02368675, 0.83076739])
    target_pick_pos = cube_pos + np.array([0,0,0.05])  # Slightly above the object
    robot.move_ee(target_pick_pos)

    # Step 2: Open Gripper Before Picking
    robot.grip(-1)  # Open the gripper

    # Step 3: Move Down to Pick Object
    target_pick_pos_down = cube_pos  # Move closer to the object
    robot.move_ee(target_pick_pos_down)

    # Step 4: Close Gripper to Pick
    robot.grip(1)  # Close gripper (grasp)

    # Step 5: Lift the Object
    robot.move_ee(target_pick_pos)  # Move back up with object

    # Step 6: Move to Drop Position
    target_drop_pos = cube_pos + np.array([0.1,0.1,0])  # Example drop-off location
    robot.move_ee(target_drop_pos)

    # Step 7: Release Object
    robot.grip(-1)  # Open gripper to release

    # Step 8: Move Back to Safe Position
    robot.move_ee(np.array([0.0, 0.0, 0.2]))  # Move to a neutral position


    # while True:
    #     command = input("Enter command (or 'exit' to quit): ")
    #     if command.lower() == "exit":
    #         break
    #     robot.execute_task(command)

    robot.env.close()

