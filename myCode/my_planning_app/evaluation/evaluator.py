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
        self.action = action
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
        self.action = self.action.lower()

        interested_pos = self._track_object_positions(self.obj_specs_history, self.interested_obj)
        obj_name = next((k for k, v in self.obj_mapping.items() if v == self.interested_obj), None)
        assert obj_name is not None, f"No object found for: '{self.interested_obj}'"

        obj_pos = interested_pos[obj_name][-1]

        if self.action == "deliver":
            return self._position_check(obj_pos=obj_pos, target_pos=self.success_criteria)

        elif self.action == "lift":
            return self._height_check(obj_pos=obj_pos, target_pos=self.success_criteria)

        else:
            raise ValueError(f"Action '{self.action}' is not supported for basic evaluation.")

    def _position_check(self, obj_pos, target_pos, threshold=0.0125):
        """
        Check if the object position is within a certain threshold of the target position.

        Parameters:
        ----------
        obj_pos (list): The current position of the object as [x, y, z].
        target_pos (list): The target position as [x, y, z].
        threhold (float): The threshold distance to check against.
        """
        # calculate the norm of the difference vector
        distance = np.linalg.norm(np.array(obj_pos) - np.array(target_pos))
        return distance <= threshold
    
    def _track_object_positions(self, obj_pos_history, query):
        """
        Track the positions of objects matching a query like 'red cube' or 'blue cylinder'.

        Args:
            obj_pos_history (list): A list of object dictionaries over time.
            query (str): A string in the format 'color shape', e.g., 'red cube'.

        Returns:
            dict: A dictionary mapping object names (obj0, obj1, ...) to a list of positions across time steps.
        """
        if not isinstance(query, str) or len(query.strip().split()) != 2:
            raise ValueError(f"Invalid query format: '{query}'. Expected format like 'red cube'.")
        color, shape = query.lower().split()
        tracked_positions = {}

        for timestep in obj_pos_history:
            for obj_name, obj_info in timestep.items():
                if obj_info["color"].lower() == color and obj_info["shape"].lower() == shape:
                    if obj_name not in tracked_positions:
                        tracked_positions[obj_name] = []
                    tracked_positions[obj_name].append(obj_info["position"])
        
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
