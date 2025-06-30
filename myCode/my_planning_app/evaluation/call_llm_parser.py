import openai
import json

def call_llm_parser(full_prompt: str, model="gpt-4-turbo") -> dict:
    """
    Calls the OpenAI API with a fully constructed prompt (from build_prompt).
    
    Args:
        full_prompt: The full instruction + context prompt string.
        model: The OpenAI model to use (default is gpt-4-turbo).

    Returns:
        A parsed JSON dictionary extracted from the LLM response.
    """
    try:
        response = openai.ChatCompletion.create(
            model=model,
            messages=[
                {"role": "user", "content": full_prompt}
            ],
            temperature=0,
            max_tokens=512
        )

        content = response["choices"][0]["message"]["content"]

        # Attempt to parse the output as JSON
        parsed = json.loads(content.strip())
        return parsed

    except json.JSONDecodeError as jde:
        print(f"[JSON Decode Error] Could not parse output: {content}")
        return {"task_type": "unknown", "error": str(jde)}

    except Exception as e:
        print(f"[LLM Parser Error] {e}")
        return {"task_type": "unknown", "error": str(e)}
