import numpy as np
import time
from robosuite import make


class RobotSkillset:
    def __init__(self, controller_config, env_name="Lift", robot="Panda", has_renderer=True):
        self.env = make(
            env_name=env_name,
            robots=robot,
            controller_configs=controller_config,
            has_renderer=has_renderer,
        )
        self.initial_ee_pos = None
        self.objects = {}

    def reset(self):
        obs = self.env.reset()
        self.initial_ee_pos = obs.get("robot0_eef_pos", None)
        return obs

    def close(self):
        self.env.close()

    def get_object_pos(self, object_name, obs=None):
        """ Helper to get object position from observation. """
        if obs is None:
            obs = self.env._get_observations()
        key = f"{object_name}_pos"
        pos = obs.get(key, None)
        if pos is None:
            raise ValueError(f"Object '{object_name}' position not found in observation!")
        return pos

    def move_ee(self, target_pos, grip_action=0, threshold=0.01, max_steps=500):
        """ Move end-effector to a target position with optional gripping action. """
        for _ in range(max_steps):
            obs = self.env._get_observations()
            current_pos = obs["robot0_eef_pos"]
            error = target_pos - current_pos

            action = np.concatenate([error * 3, [0, 0, 0], [grip_action]])
            obs, _, _, _ = self.env.step(action)
            self.env.render()

            if np.linalg.norm(error) < threshold:
                return True
        return False

    def grip(self, state, repeat=10):
        """ Control gripper: -1 (open), 1 (close) """
        action = np.array([0, 0, 0, 0, 0, 0, state])
        for _ in range(repeat):
            self.env.step(action)
            self.env.render()

    def pick(self, object_pos):
        """ Pick an object at the specified position. """
        hover_pos = object_pos + np.array([0, 0, 0.1])
        self.move_ee(hover_pos)
        self.grip(-1)
        time.sleep(0.5)
        self.move_ee(object_pos)
        self.grip(1)
        time.sleep(0.5)
        self.move_ee(hover_pos)

    def place(self, target_pos):
        """ Place the object at the target position. """
        hover_pos = target_pos + np.array([0, 0, 0.1])
        self.move_ee(hover_pos)
        self.move_ee(target_pos)
        self.grip(-1)
        time.sleep(0.5)
        self.move_ee(hover_pos)

    def return_home(self):
        """ Return end-effector to initial position. """
        if self.initial_ee_pos is not None:
            self.move_ee(self.initial_ee_pos)

    def execute_symbolic_plan(self, plan, object_positions):
        """
        plan: list of symbolic steps like:
              [("pick", "red_cube"), ("place", "blue_block")]
        object_positions: dict mapping object names to 3D positions
        """
        for step in plan:
            action = step[0]
            if action == "pick":
                obj = step[1]
                self.pick(object_positions[obj])
            elif action == "place":
                target = step[1]
                self.place(object_positions[target])
            elif action == "move":
                obj = step[1]
                where = step[2]
                if where == "above":
                    self.move_ee(object_positions[obj] + np.array([0, 0, 0.1]))
                elif where == "contact":
                    self.move_ee(object_positions[obj])
                elif where == "place_location":
                    self.move_ee(object_positions[obj])
                elif where == "above_location":
                    self.move_ee(object_positions[obj] + np.array([0, 0, 0.1]))
            elif action == "gripper_open":
                self.grip(-1)
            elif action == "gripper_close":
                self.grip(1)
            elif action == "lift":
                obj = step[1]
                self.move_ee(object_positions[obj] + np.array([0, 0, 0.1]))
