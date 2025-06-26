# prompt_engine/llm_interface.py
import openai
import os
import ast
from prompt_engine.utils import extract_symbolic_plan, extract_explanation_block, extract_waypoints

def call_llm(prompt: str, task_type: str) -> dict:
    openai.api_key = os.getenv("OPENAI_API_KEY")

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.5
    )

    content = response["choices"][0]["message"]["content"]
    print("-----RAW LLM RESPONSE-----")
    print(content)
    print("---------------------------")
    explanation = extract_explanation_block(content)
    
    if task_type in ["basic", "ambiguous"]:
        symbolic_plan = extract_symbolic_plan(content)

        # No plan given - return a clarification message
        if not symbolic_plan:
            return {
                "message": content,
                "symbolic_plan": None,
                "needs_clarification": True
            }
    
    elif task_type == "trajectory":
        trajectory_points = extract_waypoints(content)
        return {
            "explanation": explanation,
            "trajectory_points": trajectory_points,
        }
    
    else:
        raise ValueError(f"Unknown task type: {task_type}, expected 'basic', 'ambiguous', or 'trajectory'.")
    
    return {
        "explanation": explanation,
        "symbolic_plan": symbolic_plan
    }
