import numpy as np
import time

class SkillExecutor:
    def __init__(self, env):
        self.env = env
        self.initial_pos = env._get_observations()["robot0_eef_pos"]

    def execute_plan(self, plan):
        if plan is None:
            print("[ERROR] Received None as the symbolic plan. Skipping execution.")
            return
        for step in plan:
            if not isinstance(step, list) or not step:
                continue
            if isinstance(step[0], str) and step[0].startswith("//"):
                continue
            print(f"Executing: {step}")
            getattr(self, f"do_{step[0]}")(*step[1:])

    def do_move(self, target):
        """ Move to an object or a specific coordinate. """
        above_pos = np.array([0, 0, 0.15])  # Default above position
        if isinstance(target, str):
            target_pos = self._get_obj_position(target)
            self._move_ee(above_pos + target_pos)  # Move above the object
            target_pos[2] += 0.07  # Move above the object
        else:
            # fill the index 2 as 0.8 as default Z position if the target is a coordinate as two elements list
            if len(target) == 2:
                target = list(target) + [0.8]
            target_pos = np.array(target)
            self._move_ee(above_pos + target_pos)  # Move above the coordinate
            target_pos[2] += 0.025 # Adjust Z to be above the target position to avoid collision with the table
        self._move_ee(target_pos)
    
    def do_gripper_move(self, target):
        target_pos = np.array(target)
        self._move_ee(target_pos)

    def do_grip_and_pickup(self, obj_name):
        """ descend to grip(contact_pos), close gripper, then lift(above_pos). """
        obj_pos = self._get_obj_position(obj_name)
        above_pos = obj_pos + np.array([0, 0, 0.15])
        contact_pos = obj_pos

        self._grip(-1)  # open gripper
        self._move_ee(contact_pos)
        self._grip(1)
        self._move_ee(above_pos)  # lift after gripping

    def do_gripper_open(self):
        self._grip(-1)

    def do_gripper_close(self):
        self._grip(1)

    def _get_obj_position(self, name):
        try:
            body_id = self.env.sim.model.body_name2id(name + '_main')  # Add '_main' suffix for Mujoco body name
            pos = self.env.sim.data.body_xpos[body_id]
            return np.array(pos)
        except Exception as e:
            raise ValueError(f"Unknown object name in Mujoco simulation: {name}") from e

    def _move_ee(self, target_pos, grip_action=0, threshold=0.01, max_steps=500):
        """ Move end-effector to a target position with optional gripping action. """
        for _ in range(max_steps):
            obs = self.env._get_observations()
            current_pos = obs["robot0_eef_pos"]
            error = target_pos - current_pos

            action = np.concatenate([error * 3, [0, 0, 0], [grip_action]])
            self.env.step(action)
            if self.env.viewer is not None:
                self.env.render()

            if np.linalg.norm(error) < threshold:
                return True
        print(f"[WARN] Failed to reach {target_pos}")
        return False

    def _grip(self, state):
        action = np.array([0, 0, 0, 0, 0, 0, state])
        for _ in range(10):
            self.env.step(action)
            if self.env.viewer is not None:
                self.env.render()
        time.sleep(1)

    def idle(self, mode='release', duration=1):
        """ 
        Move the gripper 0.3m above the current position, 
        then keep the robot idle at that position for the specified duration.
        """
        # Get current gripper position
        obs = self.env._get_observations()
        current_pos = np.array(obs["robot0_eef_pos"])

        if mode == 'release':
            # Open the gripper
            self._grip(-1)
            # Compute target release idle position (0.3m above)
            target_pos = current_pos + np.array([0, 0, 0.3])

            # Move the end-effector to the idle position
            success = self._move_ee(target_pos)
            if not success:
                print("[WARN] Could not reach idle position, staying at current location.")
        elif mode == 'hold':
            # Hold the current position
            target_pos = current_pos
        else:
            raise ValueError("Invalid mode. Use 'release' or 'hold'.")
        
        # Only continue if environment is still running
        if getattr(self.env, "done", False):
            print("[INFO] Environment is terminated. Skipping idle step.")
            return
        
        # Hold position (send zero action) for the duration
        action = np.zeros(self.env.action_dim)
        steps = int(duration * 500)
        for _ in range(steps):
            self.env.step(action)
            if self.env.viewer is not None:
                self.env.viewer()
            self.env.render()
            time.sleep(0.01)
    
    def get_all_object_descriptions(self):
        """
        Return a dictionary mapping object names to their full description,
        including name, shape, color, position, and size — suitable for ObjectSpec.
        """
        object_descriptions = {}
        for obj in self.env.object_metadata:
            name = obj["name"]
            shape = obj.get("shape", "cube")  # fallback
            color = obj.get("color", "red")   # fallback
            size = obj.get("size", 0.025)

            body_name = name + "_main"
            if body_name in self.env.sim.model.body_names:
                body_id = self.env.sim.model.body_name2id(body_name)
                pos = self.env.sim.data.body_xpos[body_id]
                object_descriptions[name] = {
                    "name": name,
                    "shape": shape,
                    "color": color,
                    "position": pos.tolist(),
                    "size": size
                }
        return object_descriptions
    
    # def reset_robot(self):
    #     """
    #     Reset the robot to its initial position without resetting the environment.
    #     This is useful for reusing the same environment instance.
    #     """
    #     self.env._reset_robot()
    #     self.env._set_robot_pos(self.initial_pos)
    #     self.env._set_robot_quat([1, 0, 0, 0])