# prompt_engine/sim_wrapper.py
from myCode.skill_executor import SkillExecutor
from myCode.config_controller import controller
from myCode.my_env.multi_object_lift import MultiObjectLift
from myCode.config_controller import controller
from myCode.my_planning_app.prompt_engine.prompt_loader import load_default_prompt
from myCode.my_planning_app.prompt_engine.models import ObjectSpec

class SimWrapper:
    def __init__(self, scene_config_path=None):
        # Initialize the environment with the controller configuration
        self.scene_config_path = scene_config_path
        self.env = MultiObjectLift(
            robots="Panda",
            controller_configs=controller,
            has_renderer=True,
            scene_config_path = self.scene_config_path,
            ignore_done = True
        )
        self.env.reset()

        for body_name in self.env.sim.model.body_names:
            if "cube_main" in body_name:
                raise ValueError(f"The default cube '{body_name}' still exists in the environment.")
        self.executor = SkillExecutor(self.env)

    def get_current_state(self):
        return self.executor.get_all_object_descriptions()

    def execute_plan(self, plan):
        """
        1. Execute a symbolic plan in the MuJoCo simulation.
        2. Returns the history of object positions after execution.
        """

        print("\nLLM generated plan:\n", plan)
        obj_specs_history = self.executor.execute_plan(plan)

        return obj_specs_history

    # def set_object_pose(self, object_name, position, orientation = None):
    #     """
    #     Sets the pose of an object in the MuJoCo simulation by name.
    #     Args:
    #         object_name (str): name of the object (must match mujoco body name).
    #         position (list): [x, y, z]
    #         orientation (list): [x, y, z, w] quaternion
    #     """
    #     sim = self.env.sim

    #     if orientation is None:
    #         orientation = [0, 0, 0, 1]

    #     try:
    #         body_id = sim.model.body_name2id(object_name)
    #         sim.model.body_pos[body_id] = position
    #         sim.model.body_quat[body_id] = orientation
    #         sim.forward()
    #         print(f"✅ Object '{object_name}' pose set to {position}, {orientation}")
    #     except Exception as e:
    #         print(f"❌ Failed to set pose for '{object_name}': {e}")


    # def reset_scene(self):
    #     """
    #     Reload the initial scene setup without restarting the viewer or creating a new instance.
    #     """
    #     try:
    #         # Load initial scene from config again
    #         base_prompt = load_default_prompt(self.scene_config_path)
    #         raw_objects = base_prompt.environment.get("objects", [])
    #         objects = [ObjectSpec(**obj) for obj in raw_objects.values()]
    #         print("debugging ...")
    #         print(f"type of objects: {type(objects)}")
    #         print(f"objects: {objects}")
    #         # Reset the environment manually
    #         self.env.reset()

    #         # Reset objects to initial positions
    #         for obj in objects:
    #             name = obj["name"]
    #             pos = obj["position"]
    #             self.set_object_pose(name, pos)
    #     except Exception as e:
    #         print(f"Error resetting scene: {e}")

    #     # Reset robot (if needed)
    #     self.reset_robot()

    #     print("🔄 SimWrapper scene reset from saved config.")

    # def reset_robot(self):
    #     self.executor.reset_robot()

    # def idle_and_close(self):
    #     time.sleep(1)
    #     self.env.close()
    #     self.env = None