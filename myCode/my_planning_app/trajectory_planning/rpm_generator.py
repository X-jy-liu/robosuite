import numpy as np
from scipy.spatial import KDTree
import networkx as nx
import matplotlib.pyplot as plt

class SceneObject:
    def __init__(self, name, position, size):
        self.name = name
        self.position = np.array(position)
        self.size = size

    def is_collision(self, point, buffer=0.02):
        return np.linalg.norm(point - self.position) <= self.size + buffer


class RPMGenerator:
    def __init__(self, objects_pos, ref_pnt_pos, max_rpm=1000):
        self.objects = [SceneObject(obj["name"], obj["position"], obj["size"]) for obj in objects_pos]
        self.ref_points = {k: np.array(v) for k, v in ref_pnt_pos.items()}
        self.max_rpm = max_rpm
        self.nodes = []
        self.graph = nx.Graph()

    def is_collision_free(self, point, ignore_obj=None):
        return all(
            not obj.is_collision(point)
            for obj in self.objects
            if obj != ignore_obj
            )

    def edge_is_collision_free(self, p1, p2, step_size=0.05, ignore_obj=None):
        vec = p2 - p1
        dist = np.linalg.norm(vec)
        if dist == 0:
            return False
        direction = vec / dist
        steps = int(dist / step_size)
        for i in range(steps + 1):
            intermediate = p1 + i * step_size * direction
            if not self.is_collision_free(intermediate, ignore_obj):
                return False
        return True

    def build(self, k_radius=0.2, bounds=([-0.3, 0.3], [-0.3, 0.3]), max_retries=5):
        retries = 0
        while retries < max_retries:
            self.nodes = []
            self.graph = nx.Graph()

            while len(self.nodes) < self.max_rpm:
                sample = np.random.uniform([bounds[0][0], bounds[1][0]], [bounds[0][1], bounds[1][1]])
                if self.is_collision_free(sample, ignore_obj=None):
                    self.nodes.append(sample)

            object_node_map = {}
            for obj in self.objects:
                obj_idx = len(self.nodes)
                self.nodes.append(obj.position)
                self.graph.add_node(obj_idx, pos=obj.position.tolist(), label=obj.name)
                object_node_map[obj.name] = obj_idx

            ref_idx_map = {}
            for name, pt in self.ref_points.items():
                idx = len(self.nodes)
                self.nodes.append(pt)
                ref_idx_map[name] = idx

            self.nodes = np.array(self.nodes)
            tree = KDTree(self.nodes[:-len(self.objects)])

            for obj in self.objects:
                obj_idx = object_node_map[obj.name]
                _, indices = tree.query(obj.position, k=5, distance_upper_bound=0.25)
                for i in np.atleast_1d(indices):
                    if i < len(self.nodes) and self.edge_is_collision_free(obj.position, self.nodes[i], ignore_obj=obj):
                        cost = np.linalg.norm(obj.position - self.nodes[i])
                        self.graph.add_edge(obj_idx, i, weight=cost)
                        break

            for i, node in enumerate(self.nodes):
                self.graph.add_node(i, pos=node.tolist())
                neighbors = tree.query_ball_point(node, k_radius)
                for j in neighbors:
                    if i != j and not self.graph.has_edge(i, j):
                        if self.edge_is_collision_free(node, self.nodes[j], ignore_obj=None):
                            cost = np.linalg.norm(node - self.nodes[j])
                            self.graph.add_edge(i, j, weight=cost)

            # Merge object and reference point indices
            important_nodes = list(object_node_map.values()) + list(ref_idx_map.values())
            # Check full connectivity across all important nodes
            if all(nx.has_path(self.graph, important_nodes[0], other) for other in important_nodes[1:]):
                return self.graph, self.nodes
            else:
                retries += 1
                print(f"[Retry {retries}] Not all objects and reference points are connected. Resampling...")

        raise RuntimeError("Failed to connect all reference points after multiple retries.")

    def visualize_prm(self, graph, nodes, objects_pos, ref_pnt_pos):
        plt.figure(figsize=(6, 6))
        for (i, j) in graph.edges():
            p1, p2 = nodes[i], nodes[j]
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'gray', linewidth=0.5)

        plt.scatter(nodes[:, 0], nodes[:, 1], s=10, c='blue', label='Sampled Nodes')

        for obj in objects_pos:
            x, y = obj["position"]
            r = obj["size"] / 2
            circle = plt.Circle((x, y), r, color='red', alpha=0.5)
            plt.gca().add_patch(circle)
            plt.plot(x, y, 'rx')
            plt.text(x, y + 0.02, obj["name"], fontsize=8, ha='center')

        for name, pos in ref_pnt_pos.items():
            x, y = pos
            plt.plot(x, y, 'go')
            plt.text(x, y - 0.02, name, fontsize=8, ha='center', color='green')

        # Optional: draw object-to-PRM edges in red
        for obj in objects_pos:
            obj_pos = np.array(obj["position"])
            for (i, j) in graph.edges():
                if np.allclose(nodes[i], obj_pos) or np.allclose(nodes[j], obj_pos):
                    other = j if np.allclose(nodes[i], obj_pos) else i
                    plt.plot([obj_pos[0], nodes[other][0]], [obj_pos[1], nodes[other][1]], 'r--', linewidth=1.0)

        plt.title("PRM with Objects and Reference Points")
        plt.xlim(-0.4, 0.4)
        plt.ylim(-0.4, 0.4)
        plt.gca().set_aspect('equal')
        plt.legend(loc='upper right')
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    objects_pos = [
        { "name": "obj0", "shape": "cube", "color": "red", "position": [-0.2, -0.2], "size": 0.05 },
        { "name": "obj1", "shape": "cube", "color": "blue", "position": [0.2, -0.2], "size": 0.05 },
        { "name": "obj2", "shape": "cube", "color": "green", "position": [-0.2, 0.2], "size": 0.05 },
        { "name": "obj3", "shape": "cylinder", "color": "red", "position": [0.15, 0.15], "size": 0.05 },
        { "name": "obj4", "shape": "cylinder", "color": "blue", "position": [0.0, 0.0], "size": 0.05 }
    ]

    ref_pnt_pos = {
        "point_1": [-0.1, -0.1],
        "point_2": [ 0.1, -0.1],
        "point_3": [ 0.0, -0.2],
        "point_4": [ 0.05, 0.2],
        "point_5": [-0.29, 0.0]
    }

    prm = RPMGenerator(objects_pos, ref_pnt_pos, max_rpm=50)
    graph, nodes = prm.build()
    print(f"outputs from prm.build(): graph in {type(graph)}, nodes in {type(nodes)}")
    # # show a sample of the graph and nodes
    # print("Graph built successfully with", len(graph.nodes), "nodes and", len(graph.edges), "edges.")
    # print("Sample nodes:", nodes[:5])
    # print("Sample edges:", list(graph.edges)[:5])
    # print(type(graph), type(nodes))
    # prm.visualize_prm(graph, nodes, objects_pos, ref_pnt_pos)
