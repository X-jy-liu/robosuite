import json

def load_objects_from_json(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)

    # Extract the list of objects from the "environment" section
    objects = data["environment"]["objects"]
    return objects

def load_dots_from_json(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)

    # Extract the list of objects from the "environment" section
    objects = data["reference_points"]
    return objects