import numpy as np

class Evaluator:
    """
    Base class for evaluators.
    """
    def __init__(self, 
                 task_type, 
                 action, 
                 interested_obj, 
                 success_criteria, 
                 init_obj_specs, 
                 obj_specs_history, 
                 obj_mapping, 
                 ref_pnt_mapping):
        """
        Initializes the Evaluator required information.
        Parameters:
        ----------
        task_type (str): Type of the task (e.g., "simple", "ambiguous", "trajectory").
        action (str): The action to be evaluated (e.g., "deliver", "lift").
        interested_obj (str): The object(s) of interest for the evaluation.
        success_criteria (any): Criteria for success evaluation which can vary based on task type.
        init_obj_specs (dict): Initial specifications of the object(s) involved in the task.
        obj_specs_history (list): Historical specifications of the object(s) during the task - list of dicts.
        obj_mapping (dict): Mapping of object names to their specifications.
        ref_pnt_mapping (dict): Mapping of reference points to their specifications.
        """
        self.task_type = task_type
        self.action = action.lower()
        self.interested_obj = interested_obj
        self.success_criteria = success_criteria
        self.init_obj_specs = init_obj_specs
        self.obj_specs_history = obj_specs_history
        self.obj_mapping = obj_mapping
        self.ref_pnt_mapping = ref_pnt_mapping

    def basic_evaluate(self):
        """
        Basic evaluation logic that can be overridden by subclasses.
        """

        interested_pos = self._track_object_positions(self.obj_specs_history, self.interested_obj)
        obj_name = next((k for k, v in self.obj_mapping.items() if v == self.interested_obj), None)
        assert obj_name is not None, f"No object found for: '{self.interested_obj}'"

        obj_pos = interested_pos[obj_name][-1]

        if self.action in ["deliver", "stack"]:
            return self._position_check(obj_pos=obj_pos, target_pos=self.success_criteria)

        elif self.action == "lift":
            return self._height_check(obj_pos=obj_pos, target_pos=self.success_criteria)

        else:
            raise ValueError(f"Action '{self.action}' is not supported for basic evaluation.")
        
    def ambiguous_evaluate(self):
        """
        Evaluation logic for ambiguous
        """
        lst_interested_obj = [part.strip() for part in self.interested_obj.split(",")]
        if len(lst_interested_obj) != 2:
            print(f"[Warning] For ambiguous evaluation, interested_obj should contain exactly two objects. But found: {self.interested_obj} of length {len(lst_interested_obj)}.")
            return False

        interested_pos = self._track_object_positions(self.obj_specs_history, lst_interested_obj)
        obj_names = [k for k, v in self.obj_mapping.items() if v in self.interested_obj]
        if len(obj_names) != 2:
            print(f"[Warning] Expected two objects for ambiguous evaluation, but found {len(obj_names)}: {obj_names}")
            return False

        init_pos = []
        final_pos = []
        for name in obj_names:
            init_pos.append(interested_pos[name][0])  # initial position
            final_pos.append(interested_pos[name][-1])  # final position

        return self._distance_change_check(init_pos=init_pos, final_pos=final_pos, mode=self.action)
    
    def trajectory_evaluate(self):
        """
        Evaluation logic for trajectory tasks.
        Read the success criteria as a list of positions: list(list(float)).
        For each position in success criteria, check if there is a position matched in the obj_specs_history in order.
        if all positions are matched in order, return True, otherwise return False.
        """
        if not isinstance(self.success_criteria, list):
            raise ValueError("For trajectory evaluation, success_criteria should be a list of positions.")
        if not all(isinstance(pos, list) and len(pos) == 2 for pos in self.success_criteria):
            raise ValueError("Each position in success_criteria should be a list of three floats [x, y].")
        # check if self.instrested_ob is a string with two words, e.g., "red cube"
        if not isinstance(self.interested_obj, str) or len(self.interested_obj.strip().split()) != 2:
            raise ValueError(f"Invalid interested_obj format: '{self.interested_obj}'. Expected format like 'red cube'.")
        
        interested_obj_pos_dict = self._track_object_positions(self.obj_specs_history, self.interested_obj)
        obj_name = next((k for k, v in self.obj_mapping.items() if v == self.interested_obj), None)
        assert obj_name is not None, f"No object found for: '{self.interested_obj}'"

        interested_obj_pos = interested_obj_pos_dict[obj_name]
        pos_index = 0
        traj_len = len(interested_obj_pos)
        for checkpoint in self.success_criteria:
            # only check the poinsitions from current pos_index onwards
            while pos_index < traj_len:
                if self._position_check(
                    obj_pos = interested_obj_pos[pos_index][:2],
                    target_pos= checkpoint
                    ):
                    print(f"Matched checkpoint {checkpoint} at position {[round(x,3) for x in interested_obj_pos[pos_index][:2]]}")
                    pos_index += 1
                    break
                pos_index += 1
            else:
                # No match found for this checkpoint in the remaining trajectory
                return False
        print(f"Success! All checkpoints matched in order for {self.interested_obj}.")
        return True

    def _position_check(self, obj_pos, target_pos, threshold=0.025):
        """
        Check if the object position is within a certain threshold of the target position.

        Parameters:
        ----------
        obj_pos (list): The current position of the object as [x, y, z] or [x,y].
        target_pos (list): The target position as [x, y, z] or [x,y].
        threhold (float): The threshold distance to check against.
        """
        # calculate the norm of the difference vector
        distance = np.linalg.norm(np.array(obj_pos) - np.array(target_pos))
        return distance <= threshold
    
    def _track_object_positions(self, obj_pos_history, queries):
        """
        Track the positions of objects matching a query like 'red cube' or 'blue cylinder'.

        Args:
            obj_pos_history (list): A list of object dictionaries over time.
            queries (str or list): A string or list of strings like 'red cube' or a list of such strings.

        Returns:
            dict: A dictionary mapping object names (obj0, obj1, ...) to a list of positions across time steps.
        """

        if isinstance(queries, str):
            queries = [queries]  # Wrap single string into a list
        
        parsed_queries = []
        for q in queries:
            if not isinstance(q, str) or len(q.strip().split()) != 2:
                raise ValueError(f"Invalid query format: '{q}'. Expected format like 'blue cylinder'.")
            color, shape = q.lower().split()
            parsed_queries.append((color, shape))

        tracked_positions = {}

        for timestep in obj_pos_history:
            for obj_name, obj_info in timestep.items():
                obj_color = obj_info.get("color", "").lower()
                obj_shape = obj_info.get("shape", "").lower()

                for color, shape in parsed_queries:
                    if obj_color == color and obj_shape == shape:
                        if obj_name not in tracked_positions:
                            tracked_positions[obj_name] = []
                        tracked_positions[obj_name].append(obj_info["position"])
                        break  # Stop checking other queries once matched
        
        return tracked_positions
    
    def _height_check(self, obj_pos, target_pos):
        """
        Check if the object height is within a certain threshold of the target height.

        Parameters:
        ----------
        obj_pos (list): The current position of the object as [x, y, z].
        target_height (float): The target height to check against.
        threshold (float): The threshold distance to check against.
        """
        # ensure within a certrain threshold for the x, y coordinates
        if not self._position_check(obj_pos[:2], target_pos[:2]):
            return False
        # Extract the z-coordinate for height
        current_height = obj_pos[2]
        
        return current_height >= target_pos[2]
    
    def _distance_change_check(self, init_pos, final_pos, mode):
        """
        check if the distance between two objects has changed according to the specified mode.
        Parameters:
        ----------
        init_post (list(list(foat)): Initial positions of the objects as [[x1, y1, z1], [x2, y2, z2]].
        final_pos (list(list(float)): Final positions of the objects as [[x1, y1, z1], [x2, y2, z2]].
        mode (str): The mode of distance change to check, either "closer" or "further".
        Returns:
        -------
        bool: True if the distance change condition is met, False otherwise.
        """
        if len(init_pos) != 2 or len(final_pos) != 2:
            raise ValueError("init_pos and final_pos should each contain exactly two positions.")

        init_distance = np.linalg.norm(np.array(init_pos[0]) - np.array(init_pos[1]))
        final_distance = np.linalg.norm(np.array(final_pos[0]) - np.array(final_pos[1]))

        if mode == "move_closer":
            return final_distance < init_distance
        elif mode == "move_further":
            return final_distance > init_distance
        else:
            raise ValueError(f"Invalid mode '{mode}'. Expected 'closer' or 'further'.")