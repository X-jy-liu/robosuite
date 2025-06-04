import re

class CommandParser:
    '''
    Parameters:
    - command: str
    - object_mapping: dict  # maps descriptive names (e.g., 'blue cube') to object IDs (e.g., 'obj1')
    
    Outputs:
    - golden_symbolic_plan: list
    '''
    def __init__(self, command, object_mapping):
        self.command = command.strip()
        self.object_mapping = object_mapping

    def resolve_object(self, description):
        # If it's already an obj name, keep it
        if description.startswith('obj'):
            return description
        
        # Otherwise, look up from mapping (like 'blue cube' → 'obj1')
        obj_id = self.object_mapping.get(description.lower())
        if obj_id:
            return obj_id
        else:
            return f"unknown({description})"

    def parse(self):
        command = self.command
        actions = []

        # Rule 1: Stack [objectA] onto [objectB]
        match = re.match(r"Stack (.+) onto (.+)", command, re.IGNORECASE)
        if match:
            descA, descB = match.groups()
            objectA = self.resolve_object(descA)
            objectB = self.resolve_object(descB)
            actions = [
                ['lift_sequence', objectA],
                ['move', objectB],
                ['gripper', 'open']
            ]
            return actions

        # Rule 2: Move [objectA] to [positionA]
        match = re.match(r"Move (.+) to (.+)", command, re.IGNORECASE)
        if match:
            descA, positionA = match.groups()
            objectA = self.resolve_object(descA)
            actions = [
                ['lift_sequence', objectA],
                ['move', positionA]
            ]
            return actions

        # Rule 3: Lift [objectA]
        match = re.match(r"Lift (.+)", command, re.IGNORECASE)
        if match:
            descA = match.group(1)
            objectA = self.resolve_object(descA)
            actions = [
                ['lift_sequence', objectA]
            ]
            return actions

        # If no rule matched
        return [['error', 'unrecognized command']]

def build_object_mapping(objects):
    mapping = {}
    for obj in objects:
        key = f"{obj['color']} {obj['shape']}".lower()
        mapping[key] = obj['name']  # e.g., 'blue cube' → 'obj1'
    return mapping

if __name__ == "__main__":
    objects = [
        {"name": "obj0", "shape": "cube", "color": "red"},
        {"name": "obj1", "shape": "cube", "color": "blue"},
        {"name": "obj2", "shape": "cube", "color": "green"},
        {"name": "obj3", "shape": "cylinder", "color": "red"},
        {"name": "obj4", "shape": "cylinder", "color": "blue"}
    ]

    object_mapping = build_object_mapping(objects)

    test_commands = [
        "Stack the blue cube onto red cylinder",
        "Move obj2 to (0.1,0.3,0.8)",
        "Lift green cube",
        "Rotate yellow pyramid"
    ]

    for cmd in test_commands:
        parser = CommandParser(cmd, object_mapping)
        plan = parser.parse()
        print(f"Command: {cmd}")
        print("Golden Symbolic Plan:", plan)
        print()
