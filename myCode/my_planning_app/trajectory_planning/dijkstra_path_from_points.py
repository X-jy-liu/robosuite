from scipy.spatial import KDTree
import networkx as nx
import numpy as np

def dijkstra_path_from_points(trajectory_points, graph, nodes):
    """
    Given a list of [x, y] points and a roadmap (graph, nodes),
    compute the full Dijkstra path through the roadmap.
    
    Returns:
        - node_path: list of node indices in the roadmap
        - waypoint_path: list of [x, y] positions
    """
    # Step 1: Find closest node for each trajectory point
    tree = KDTree(nodes)
    point_node_ids = []
    for pt in trajectory_points:
        distance, idx = tree.query(pt)
        if distance > 0.1:
            print(f"⚠️ Point {pt} is too far from any node (distance: {distance:.2f}).")
            return [], []
        point_node_ids.append(idx)

    # Step 2: Run Dijkstra between consecutive nodes
    node_path = [point_node_ids[0]]
    for i in range(len(point_node_ids) - 1):
        src = point_node_ids[i]
        dst = point_node_ids[i+1]
        try:
            partial_path = nx.dijkstra_path(graph, src, dst, weight='weight')
            # Skip the first node if it's a continuation
            if partial_path[0] == node_path[-1]:
                node_path += partial_path[1:]
            else:
                node_path += partial_path
        except nx.NetworkXNoPath:
            print(f"⚠️ No path found between nodes {src} and {dst}")
            return [], []

    # Step 3: Convert to waypoints
    waypoint_path = [nodes[i].tolist() for i in node_path]
    return node_path, waypoint_path

def build_symbolic_plan(waypoint_path):
    idx = 0
    height_0 = 0.875
    height_1 = 0.825
    const_height = 0.85
    symbolic_plan = []
    for pt in waypoint_path:
        # round to 3 decimal places
        pt = np.round(pt, 3).tolist()
        if idx == 0:
            symbolic_plan.append(['gripper_move', [pt[0], pt[1], height_0]])
            symbolic_plan.append(['gripper_open'])
            symbolic_plan.append(['gripper_move', [pt[0], pt[1], height_1]])
            symbolic_plan.append(['gripper_close'])
            symbolic_plan.append(['gripper_move', [pt[0], pt[1], const_height]])
        else:
            symbolic_plan.append(['gripper_move', [pt[0], pt[1], const_height]])
        
        idx += 1
    symbolic_plan.append(['gripper_open'])

    return symbolic_plan
