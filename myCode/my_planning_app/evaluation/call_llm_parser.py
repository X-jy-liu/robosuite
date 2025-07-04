import openai
import json
import re

# Extract first JSON object from response (robust to Markdown formatting)
def extract_json_block(text: str):
    if isinstance(text, dict):
        return text  # already parsed
    
    if not isinstance(text, str):
        raise TypeError(f"Expected string or dict, got {type(text)}")

    # Remove markdown-style wrapping
    cleaned = text.strip().strip("`").replace("json", "", 1).strip()

    # Extract the first JSON object
    match = re.search(r"\{.*\}", cleaned, re.DOTALL)
    if match:
        return json.loads(match.group(0))
    else:
        raise ValueError("No valid JSON object found.")

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
        response = extract_json_block(content)
        return response

    except json.JSONDecodeError as jde:
        print(f"[JSON Decode Error] Could not parse output: {content}")
        return {"task_type": "unknown", "error": str(jde)}

    except Exception as e:
        print(f"[LLM Parser Error] {e}")
        return {"task_type": "unknown", "error": str(e)}
