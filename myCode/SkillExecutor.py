import numpy as np
import time

class SkillExecutor:
    def __init__(self, env):
        self.env = env
        self.initial_pos = env._get_observations()["robot0_eef_pos"]

    def execute_plan(self, plan):
        for step in plan:
            print(f"Executing: {step}")
            getattr(self, f"do_{step[0]}")(*step[1:])

    def do_move(self, target):
        """ Move to an object or a specific coordinate. """
        if isinstance(target, str):
            target_pos = self._get_obj_position(target)
            target_pos[2] += 0.1  # Move above the object
        else:
            target_pos = np.array(target)
            target_pos[2] += 0.08 # Adjust Z to be above the target position to avoid collision with the table
        self._move_ee(target_pos)

    def do_lift_sequence(self, obj_name):
        """ Move above the object, descend to grip, close gripper, then lift. """
        obj_pos = self._get_obj_position(obj_name)
        above_pos = obj_pos + np.array([0, 0, 0.1])
        contact_pos = obj_pos

        self._move_ee(above_pos)
        self._grip(-1)  # open gripper
        self._move_ee(contact_pos)
        self._grip(1)
        self._move_ee(above_pos)  # lift after gripping

    def do_gripper_open(self):
        self._grip(-1)

    def do_gripper_close(self):
        self._grip(1)

    def _get_obj_position(self, name):
        obs = self.env._get_observations()
        if name == "red_cube":
            return np.array(obs["cube_pos"])
        elif name == "blue_cube":
            return np.array(obs["cube2_pos"])
        else:
            raise ValueError(f"Unknown object name: {name}")

    def _move_ee(self, target_pos, grip_action=0, threshold=0.01, max_steps=500):
        """ Move end-effector to a target position with optional gripping action. """
        for _ in range(max_steps):
            obs = self.env._get_observations()
            current_pos = obs["robot0_eef_pos"]
            error = target_pos - current_pos

            action = np.concatenate([error * 3, [0, 0, 0], [grip_action]])
            self.env.step(action)
            self.env.render()

            if np.linalg.norm(error) < threshold:
                return True
        print(f"[WARN] Failed to reach {target_pos}")
        return False

    def _grip(self, state):
        action = np.array([0, 0, 0, 0, 0, 0, state])
        for _ in range(10):
            self.env.step(action)
            self.env.render()
        time.sleep(1)

    def reset_gripper(self):
        self._move_ee(self.initial_pos)