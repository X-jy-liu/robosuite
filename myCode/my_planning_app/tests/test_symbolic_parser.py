import unittest

from prompt_engine.utils import extract_symbolic_plan  # replace with actual module name


class TestSymbolicPlanExtraction(unittest.TestCase):
    def test_basic_plan_with_comments(self):
        llm_response = """
        Explanation:
        Step 1: Identify the object.
        symbolic plan: [
            ["move", "obj3"], // move to red cube
            ["grip_and_pickup", "obj3"] // grip and lift
        ]
        """

        expected = [
            ["move", "obj3"],
            ["grip_and_pickup", "obj3"]
        ]

        result = extract_symbolic_plan(llm_response)
        self.assertEqual(result, expected)

    def test_multiline_with_empty_lines_and_indent(self):
        llm_response = """
        symbolic plan: [
            ["move", "obj3"],    // go to obj
              
            ["grip_and_pickup", "obj3"]   // pickup
        ]
        """

        expected = [
            ["move", "obj3"],
            ["grip_and_pickup", "obj3"]
        ]

        result = extract_symbolic_plan(llm_response)
        print(f"Extracted Plan:\n{result}")
        self.assertEqual(result, expected)

    def test_plan_missing(self):
        bad_input = "No plan here."
        with self.assertRaises(ValueError):
            extract_symbolic_plan(bad_input)

    def test_plan_with_syntax_error(self):
        bad_input = """
        symbolic plan: [
            ["move", "obj3",  // missing closing bracket
            ["grip_and_pickup", "obj3"]
        ]
        """
        with self.assertRaises(ValueError):
            extract_symbolic_plan(bad_input)


if __name__ == "__main__":
    unittest.main()
