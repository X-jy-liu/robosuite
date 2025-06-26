import os
import sys
import unittest
from pathlib import Path

# Make sure parent directory is in sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from trajectory_planning.dijkstra_path_from_points import dijkstra_path_from_points
from trajectory_planning.rpm_generator import RPMGenerator
from trajectory_planning.json_loader import load_objects_from_json, load_dots_from_json

class TestRoadmapPlanning(unittest.TestCase):

    def setUp(self):
        # Define scene objects and reference points
        self.objects_pos = [
            { "name": "obj0", "shape": "cube", "color": "red", "position": [-0.2, -0.2], "size": 0.05 },
            { "name": "obj1", "shape": "cube", "color": "blue", "position": [0.2, -0.2], "size": 0.05 },
            { "name": "obj2", "shape": "cube", "color": "green", "position": [-0.2, 0.2], "size": 0.05 },
            { "name": "obj3", "shape": "cylinder", "color": "red", "position": [0.15, 0.15], "size": 0.05 },
            { "name": "obj4", "shape": "cylinder", "color": "blue", "position": [0.0, 0.0], "size": 0.05 }
        ]

        self.ref_pnt_pos = {
            "point_1": [-0.1, -0.1],
            "point_2": [ 0.1, -0.1],
            "point_3": [ 0.0, -0.2],
            "point_4": [ 0.05, 0.2],
            "point_5": [-0.29, 0.0]
        }

    def test_dijkstra_path(self):
        # Build roadmap
        rmp_generator = RPMGenerator(self.objects_pos, self.ref_pnt_pos, max_rpm=50)
        graph, nodes = rmp_generator.build()

        # Plan path between two key points
        trajectory_points = [
            [0.15, 0.15],  # obj3
            [0.1, -0.1]    # point_2
        ]
        node_path, waypoint_path = dijkstra_path_from_points(trajectory_points, graph, nodes)

        # Assertions
        self.assertGreater(len(node_path), 1, "Path should have more than 1 node.")
        self.assertEqual(len(node_path), len(waypoint_path), "Node path and waypoint path lengths should match.")
        self.assertIsInstance(waypoint_path[0], list, "Waypoint should be a list of [x, y].")

        print("✅ Node Path:", node_path)
        print("✅ Waypoint Path:", waypoint_path)
    
    def test_json_loader(self):
        scene_json_path = Path.home() / "robosuite" / "myCode" / "my_planning_app" / "prompts" / "env_and_func.json"
        dots_json_path = Path.home() / "robosuite" / "myCode" / "my_planning_app" / "prompts" / "generated_dots.json"

        # Load objects from JSON
        objects = load_objects_from_json(scene_json_path)
        print(objects)
        self.assertIsInstance(objects, list, "Loaded objects should be a list.")
        self.assertGreater(len(objects), 0, "There should be at least one object in object-creating prompt.")
        # load reference points from JSON
        dots = load_dots_from_json(dots_json_path)
        print(dots)
        self.assertIsInstance(dots, dict, "Loaded dots should be a dict.")
        

if __name__ == '__main__':
    unittest.main()
