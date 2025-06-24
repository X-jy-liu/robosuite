# prompt_engine/llm_interface.py
import openai
import os
import ast
from prompt_engine.utils import extract_block

def call_llm(prompt: str):
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

    explanation = extract_block(content, "Explanation:")
    plan_str = extract_block(content, "Symbolic Plan:")

    # No plan given - return a clarification message
    if not plan_str.strip():
        return {
            "message": content,
            "symbolic_plan": None,
            "needs_clarification": True
        }
    try:
        symbolic_plan = ast.literal_eval(plan_str.strip())
    except Exception as e:
        raise RuntimeError(f"Failed to parse the symbolic plan: {e}")

    return {
        "explanation": explanation,
        "symbolic_plan": symbolic_plan
    }
