# prompt_engine/sim_wrapper.py
from myCode.skill_executor import SkillExecutor
from myCode.config_controller import controller
from myCode.my_env.multi_object_lift import MultiObjectLift

class SimWrapper:
    def __init__(self):
        self.env = MultiObjectLift(
            robots="Panda",
            controller_configs=controller,
            has_renderer=True
        )
        self.env.reset()

        for body_name in self.env.sim.model.body_names:
            if "cube_main" in body_name:
                raise ValueError(f"The default cube '{body_name}' still exists in the environment.")
        self.executor = SkillExecutor(self.env)

    def get_current_state(self):
        return self.executor.get_all_object_descriptions()

    def execute_plan(self, plan):
        print("Executing:", plan)
        self.executor.execute_plan(plan)

    def reset_robot(self):
        self.executor.reset_robot_only()

    def idle_and_close(self):
        self.executor.idle()
        self.env.close()