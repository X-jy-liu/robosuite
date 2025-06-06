import re

class TargetedPositionParser:
    '''
    Parameters:
    - command: str
    - object_mapping: dict with keys as names and values containing:
      {'id': 'obj1', 'shape': 'cube', 'height': 0.05, 'position': (x, y, z)}

    Output:
    - final_target_position: tuple or error string
    '''
    def __init__(self, command, object_mapping, above_position=0.15):
        self.command = command.strip()
        self.object_mapping = object_mapping
        self.tolerance = 0.00625  # Default tolerance for objects' final positions
        self.required_keys = {'id', 'shape', 'height', 'position'}
        self.above_position = above_position  # Default lift height is 0.15

    def resolve_object(self, description):
        if description.startswith('obj'):
            for v in self.object_mapping.values():
                if v['id'] == description:
                    return v
        return self.object_mapping.get(description.lower(), None)
    
    def validate_object_mapping(self):
        for name, obj in self.object_mapping.items():
            missing = self.required_keys - obj.keys()
            if missing:
                raise ValueError(f"Object '{name}' is missing keys: {missing}")
    
    def apply_tolerance(self, center, tolerance):
        """        
        Apply tolerance to the center position of an object.
        """
        return {
            "target_center": center,
            "tolerance": tolerance,
            "range": {
                "x": [center[0] - tolerance, center[0] + tolerance],
                "y": [center[1] - tolerance, center[1] + tolerance],
                "z": [center[2] - tolerance, center[2] + tolerance],
            }
        }

    def parse(self):
        command = self.command

        # Rule 1: Stack A onto B
        match = re.match(r"Stack (?:the )?(.+) onto (.+)", command, re.IGNORECASE)
        if match:
            descA, descB = match.groups()
            objectA = self.resolve_object(descA)
            objectB = self.resolve_object(descB)
            if objectA and objectB:
                offset_z = 0.5 * objectA.get('height')
                x, y, z = objectB['position'] # z is the height of objectB's geometric center
                target_position = (x, y, z + offset_z)
                res = self.apply_tolerance(target_position, self.tolerance)
                return res
            return {"error": "unrecognized command"}

        # Rule 2: Move A to (x,y,z)
        match = re.match(r"Move (.+) to \((.+)\)", command, re.IGNORECASE)
        if match:
            descA, pos_str = match.groups()
            objectA = self.resolve_object(descA)
            position = tuple(map(float, pos_str.split(',')))
            
            if objectA:
                res = self.apply_tolerance(position, self.tolerance)
                return res
            return {"error": "unrecognized command"}

        # Rule 3: Lift A
        match = re.match(r"Lift (.+)", command, re.IGNORECASE)
        if match:
            descA = match.group(1)
            objectA = self.resolve_object(descA)
            if objectA:
                x, y, z = objectA['position']
                target_position = (x,y,z + self.above_position)  # Lift by 0.15 meters
                return self.apply_tolerance(target_position, self.tolerance)
            return {"error": "unrecognized command"}

        return {"error": "unrecognized command"}

def build_object_mapping(objects):
    mapping = {}
    for obj in objects:
        key = f"{obj['color']} {obj['shape']}".lower()
        mapping[key] = {
            'id': obj['name'],
            'shape': obj['shape'],
            'height': obj.get('height', 0.05),
            'position': obj.get('position', (0, 0, 0))
        }
    return mapping

if __name__ == "__main__":
    objects = [
        {"name": "obj0", "shape": "cube", "color": "red", "position": (0.1, 0.1, 0.0)},
        {"name": "obj1", "shape": "cube", "color": "blue", "position": (0.2, 0.2, 0.0)},
        {"name": "obj2", "shape": "cube", "color": "green", "position": (0.3, 0.3, 0.0)},
        {"name": "obj3", "shape": "cylinder", "color": "red", "position": (0.4, 0.4, 0.0)},
        {"name": "obj4", "shape": "cylinder", "color": "blue", "position": (0.5, 0.5, 0.0)}
    ]

    object_mapping = build_object_mapping(objects)

    test_commands = [
        "Stack the blue cube onto red cylinder",
        "Move obj2 to (0.1,0.3,0.8)",
        "Lift green cube",
        "Rotate yellow pyramid"
    ]

    for cmd in test_commands:
        parser = TargetedPositionParser(cmd, object_mapping)
        pos = parser.parse()
        print(f"Command: {cmd}\n")
        if 'error' in pos:
            print(f"→ Error: {pos['error']}\n")
        else:
            print(f"→ Final Target Position: \n target position: {pos['target_center']} \n x:{pos['range']['x']} \n y:{pos['range']['y']} \n z:{pos['range']['z']}\n")
            print(f" upper tolerance of x: {pos['range']['x'][1]} \n lower tolerance of x: {pos['range']['x'][0]}")