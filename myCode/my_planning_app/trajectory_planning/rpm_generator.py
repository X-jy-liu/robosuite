import numpy as np
from scipy.spatial import KDTree
import networkx as nx
import matplotlib.pyplot as plt

class SceneObject:
    """
    Represents an object in the scene with a position, size, and buffer for collision checking.

    Parameters
    ----------
    name : str
        Name identifier for the object.
    position : list or np.ndarray
        2D coordinates of the object center.
    size : float
        Radius (or half-size) of the object.
    buffer : float
        Additional safety margin added to the size for collision detection.

    Methods
    -------
    is_collision(point)
        Checks whether a given point is in collision with the object (within size + buffer).
    """

    def __init__(self, name, position, size, buffer):
        self.name = name
        self.position = np.array(position)
        self.size = size
        self.buffer = buffer

    def is_collision(self, point):
        return np.linalg.norm(point - self.position) <= self.size + self.buffer


class RPMGenerator:
    """
    Generates a Probabilistic Roadmap (PRM) graph for path planning between scene objects
    and reference points, ensuring collision-free connections.

    Parameters
    ----------
    obj_specs : list of dict
        Each dictionary must include 'name', 'position', and 'size' keys to describe scene objects.
    pnt_specs : dict
        Dictionary mapping reference point names to their 2D positions (as lists or arrays).
    roadmap_buffer : float
        Additional margin added to object size for collision avoidance.
    max_rpm : int
        Maximum number of sampled nodes in the roadmap.

    Attributes
    ----------
    objects : list of SceneObject
        List of initialized scene objects with collision models.
    ref_points : dict
        Mapping from reference point names to numpy arrays of coordinates.
    max_rpm : int
        Maximum number of roadmap nodes.
    nodes : list of np.ndarray
        Sampled PRM node positions.
    graph : networkx.Graph
        PRM graph where nodes are positions and edges are feasible paths.

    Methods
    -------
    is_collision_free(point, ignore_obj=None)
        Checks whether a point is collision-free relative to all scene objects.
    
    edge_is_collision_free(p1, p2, step_size=0.05, ignore_obj=None)
        Determines if a straight-line edge between two points is collision-free.
    
    build(roadmap_bounds=[-0.28, 0.28], k_radius=0.2, max_retries=5)
        Constructs the PRM graph, retrying if full connectivity between objects and ref points fails.
    
    visualize_prm(graph, nodes, objects_pos, ref_pnt_pos)
        Visualizes the roadmap and its connections, objects, and reference points using matplotlib.
    """
    def __init__(self, obj_specs, pnt_specs, roadmap_buffer, max_rpm):
        self.objects = [SceneObject(obj["name"], obj["position"], obj["size"], roadmap_buffer) for obj in obj_specs]
        self.ref_points = {k: np.array(v) for k, v in pnt_specs.items()}
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

    def build(self, roadmap_bounds=[-0.28, 0.28], k_radius=0.2, max_retries=10):
        retries = 0
        sample_attempts = 0
        max_sample_attempts = 1000  # Limit to avoid infinite loops
        while retries < max_retries:
            print(f"[Attempt {retries + 1}] Building PRM with max_rpm={self.max_rpm}, roadmap_bounds={roadmap_bounds}, k_radius={k_radius}")
            self.nodes = []
            self.graph = nx.Graph()

            while len(self.nodes) < self.max_rpm and sample_attempts < max_sample_attempts:
                sample = np.random.uniform(roadmap_bounds[0], roadmap_bounds[1], size=2)
                sample_attempts += 1
                if sample_attempts % 100 == 0:
                    # Print progress every 100 attempts
                    print(f"On {retries+1}/{max_retries} retries: {sample_attempts}th attempts")
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
                print(f"[Success] All objects and reference points are connected after {retries + 1} attempts.")
                return self.graph, self.nodes
            else:
                retries += 1
                print(f"[Retry {retries}] Not all objects and reference points are connected. Resampling...")
        
        if retries == max_retries:
            print("Failed to connect all reference points after multiple retries.")
            raise ValueError("Failed to generate a valid roadmap graph or nodes after maximum retries.")

    def visualize_prm(self, graph, nodes, obj_specs, pnt_specs, ax=None):
        """
        Visualizes the PRM roadmap, objects, and reference points.

        Parameters
        ----------
        graph : networkx.Graph
            The PRM graph with edge connections.
        nodes : np.ndarray
            Node coordinates.
        obj_specs : list of dict
            Each dict must have 'name', 'position', and 'size' keys.
        pnt_specs : dict
            Mapping from point name to [x, y] coordinates.
        ax : matplotlib.axes.Axes, optional
            If provided, plot on this axis instead of the global one.
        """

        if ax is None:
            fig, ax = plt.subplots(figsize=(6, 6))

        try:
            # Plot edges
            for (i, j) in graph.edges():
                p1, p2 = nodes[i], nodes[j]
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'gray', linewidth=0.3, alpha=0.2, zorder=1)

            # Plot roadmap nodes
            ax.scatter(nodes[:, 0], nodes[:, 1], s=10, c='blue', label='Sampled Nodes', zorder=3)

            # Plot objects
            for obj in obj_specs:
                y, x = obj["position"]
                r = obj["size"] / 2
                circle = plt.Circle((x, y), r, color='red', alpha=0.5)
                ax.add_patch(circle)
                ax.plot(x, y, 'rx')
                ax.text(x, y + 0.02, obj["name"], fontsize=8, ha='center')

            # Plot reference points
            for name, pos in pnt_specs.items():
                x,y = pos
                ax.plot(x, y, 'go')
                ax.text(x, y - 0.02, name, fontsize=8, ha='center', color='green', zorder=4)

            # Optional: highlight object-to-node connections
            for obj in obj_specs:
                obj_pos = np.array(obj["position"])
                for (i, j) in graph.edges():
                    if np.allclose(nodes[i], obj_pos) or np.allclose(nodes[j], obj_pos):
                        other = j if np.allclose(nodes[i], obj_pos) else i
                        ax.plot([obj_pos[0], nodes[other][0]], [obj_pos[1], nodes[other][1]], 'r--', linewidth=1.0)

            # Final layout
            ax.set_title("PRM with Objects and Reference Points")
            ax.set_xlim(-0.4, 0.4)
            ax.set_ylim(0.4, -0.4)
            ax.set_aspect('equal')
            ax.grid(True)
            ax.legend(loc='upper right')


            # Show only if not embedded in external context
            if ax is None:
                plt.show()

        except Exception as e:
            print(f"[visualize_prm] Error during visualization: {e}")



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
