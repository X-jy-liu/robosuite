# prompt_engine/utils.py
def extract_block(text: str, header: str) -> str:
    lines = text.splitlines()
    found = False
    block = []
    for line in lines:
        if found:
            if line.strip() == "":
                break
            block.append(line)
        elif line.strip().startswith(header):
            found = True
    return "\n".join(block).strip()

def format_examples(example_list):
    if not example_list:
        return ""
    
    formatted = []
    for ex in example_list:
        ex_task = ex.get("task", "")
        ex_plan = ex.get("plan", [])
        ex_explanation = ex.get("explanation", "")

        plan_str = "\n    ".join([str(tuple(step)) for step in ex_plan])
        block = f'Example Task: "{ex_task}"\n  Plan:\n    {plan_str}'

        if ex_explanation:
            indented_explanation = ex_explanation.replace("\n", "\n    ")
            block += "\n  Explanation:\n    " + indented_explanation

        formatted.append(block)

    return "\n\n" + "\n\n".join(formatted)

