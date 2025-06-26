import unittest
from prompt_engine.prompt_builder import construct_prompt

class TestPromptBuilder(unittest.TestCase):
    def test_construct_basic_prompt(self):
        command = "lift the green cube"
        task_type = "basic"
        prompt = construct_prompt(command=command, task_type=task_type)

        self.assertIn("lift the green cube", prompt)
        self.assertIn("Example Scene and reference points (if applicable):", prompt)
        self.assertIn("Available Functions:", prompt)
        self.assertIn("Examples:", prompt)
        self.assertIn("Now, generate a symbolic plan", prompt)
        self.assertIn("Instructions:", prompt)

    def test_construct_ambiguous_prompt(self):
        command = "put the red cube and green cube together"
        task_type = "ambiguous"
        prompt = construct_prompt(command=command, task_type=task_type)

        self.assertIn("put the red cube and green cube together", prompt)
        self.assertIn("Instructions:", prompt)
        self.assertIn("Examples:", prompt)
        self.assertIn("Example Scene and reference points (if applicable):", prompt)
        self.assertIn("Current Scene and reference points (if applicable):", prompt)
    def test_chain_mode_prompt(self):
        prompt = construct_prompt(
            command="move it closer to the blue cube",
            task_type="ambiguous",
            mode="chain",
            initial_command="lift the red cube"
        )
        self.assertIn("Original Task: lift the red cube", prompt)
        self.assertIn("Follow-up Instruction: move it closer to the blue cube", prompt)

if __name__ == "__main__":
    unittest.main()
