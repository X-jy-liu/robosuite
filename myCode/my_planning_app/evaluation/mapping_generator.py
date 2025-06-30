def obj_name_mapping(obj_info):
    """
    Create a mapping from object names (e.g., 'obj0') to descriptive strings like 'green cube'.

    Args:
        obj_info (list): List of object dictionaries. Each dict describes one object's information.

    Returns:
        dict: Mapping like {'obj0': 'green cube', ...}
    """
    if not obj_info:
        return {}

    mapping = {}
    for obj in obj_info:
        obj_name = obj.get("name", "")
        obj_color = obj.get("color", "").lower()
        obj_shape = obj.get("shape", "").lower()
        if obj_name and obj_color and obj_shape:
            mapping[obj_name] = f"{obj_color} {obj_shape}"
    return mapping

if __name__=="__main__":
    lst = [{'name': 'obj0', 'shape': 'cube', 'color': 'green', 'position': [0.146, 0.005], 'size': 0.05}, {'name': 'obj1', 'shape': 'cube', 'color': 'red', 'position': [-0.019, -0.039], 'size': 0.05}, {'name': 'obj2', 'shape': 'cube', 'color': 'red', 'position': [-0.169, -0.143], 'size': 0.05}, {'name': 'obj3', 'shape': 'cube', 'color': 'blue', 'position': [0.083, 0.145], 'size': 0.05}, {'name': 'obj4', 'shape': 'cylinder', 'color': 'red', 'position': [0.044, -0.206], 'size': 0.05}]
    mapping = obj_name_mapping(lst)
    print(mapping)