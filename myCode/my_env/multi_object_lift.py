#TODO: enable to take the input of different layouts (five objects) 

# from robosuite.environments.manipulation.lift import Lift
from myCode.my_env.lift_without_default_cube import Lift
from robosuite.models.objects import BoxObject
from robosuite.models.objects.primitive.cylinder import CylinderObject
from robosuite.utils.mjcf_utils import add_to_dict
from pathlib import Path
import json

class MultiObjectLift(Lift):
    def __init__(self, scene_config_path=None, robot_offset=False, **kwargs):
        self.scene_config_path = scene_config_path
        self.robot_offset = robot_offset
        super().__init__(**kwargs)

    def _load_model(self):
        super()._load_model()
        table_z = self.table_offset[2]
        if self.robot_offset:
            self.robots[0].robot_model.set_base_xpos([-2, 0, 0])
        half_height = 0.025  # Half height for objects
        cylinder_calibration = 0.0 # Calibration offset for cylinder height
        # checking the height of the table
        self.object_metadata = []  # Store object info for later access
        self.COLORS = {
            "red": [1, 0, 0, 1],
            "green": [0, 1, 0, 1],
            "blue": [0, 0, 1, 1]
        }

        # Predefined objects: (name, shape, color_name, position)
        if self.scene_config_path and Path(self.scene_config_path).exists():
            with open(self.scene_config_path, "r") as f:
                config = json.load(f)

            object_list = config["environment"]["objects"]

            predefined_objects = []
            for obj in object_list:
                name = obj["name"]
                shape = obj["shape"]
                color = obj["color"]
                x, y = obj["position"]

                if shape == "cube":
                    z = table_z + half_height
                else:  # cylinder
                    z = table_z + half_height + cylinder_calibration

                position = [x, y, z]
                predefined_objects.append((name, shape, color, position))
        else:
            # Fallback if no file or invalid path
            predefined_objects = [
                ("obj0", "cube", "red", [-0.2, -0.2, table_z + half_height]),
                ("obj1", "cube", "blue", [0.2, -0.2, table_z + half_height]),
                ("obj2", "cube", "green", [-0.2, 0.2, table_z + half_height]),
                ("obj3", "cylinder", "red", [0.15, 0.15, table_z + half_height + cylinder_calibration]),
                ("obj4", "cylinder", "blue", [0.0, -0.0, table_z + half_height + cylinder_calibration]),
            ]


        for name, shape, color_name, pos in predefined_objects:
            color_rgba = self.COLORS[color_name]

            if shape == "cube":
                obj = BoxObject(
                    name=name,
                    size=[0.025, 0.025, 0.025],
                    rgba=color_rgba,
                    material=None,
                    obj_type="all"
                )
            else:
                obj = CylinderObject(
                    name=name,
                    size=[0.025, 0.025],  # (radius, height)
                    rgba=color_rgba,
                    material=None,
                    obj_type="all"
                )

            obj.get_obj().set("pos", f"{pos[0]} {pos[1]} {pos[2]}")

            # Add to the model
            self.model.merge_objects([obj])
            add_to_dict(self.model.worldbody, "body", obj.get_obj())

            # save the object metadata
            self.object_metadata.append({
                "name": name,
                "shape": shape,
                "color": color_name,
                "position": pos
            })

    def _setup_camera(self):
        cam_id = self.sim.model.camera_name2id("birdview")
        self.sim.model.cam_pos[cam_id] = [0.0, 0.0, 1.5]  # Adjust height as needed
        self.sim.model.cam_quat[cam_id] = [1.0, 0.0, 0.0, 0.0]  # Look straight down
        self.sim.model.cam_fovy[cam_id] = 60  # Smaller FOV to zoom in tighter

    # def _reset_robot(self):
    #     """
    #     Reset the robot joint positions and gripper state without resetting the entire environment.
    #     Intended to be called when object layout remains unchanged but robot should return to rest.
    #     """
    #     # Reset robot joint positions to their default (rest) state
    #     self.robots[0].reset(deterministic=True)

    #     # Set gripper state to open
    #     if hasattr(self.robots[0], "gripper"):
    #         self.robots[0].gripper.set_action(-1)  # Fully open

    #     # Sync simulation data with these new initial states
    #     self.sim.forward()

