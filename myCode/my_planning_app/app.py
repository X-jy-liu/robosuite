from fastapi import FastAPI
from prompt_engine.prompt_loader import load_default_prompt
from prompt_engine.prompt_builder import construct_prompt
from prompt_engine.models import PromptInput, ObjectSpec
from prompt_engine.llm_interface import call_llm
from prompt_engine.sim_wrapper import SimWrapper
from prompt_engine.scene_checker import check_scene_consistency
from pathlib import Path

app = FastAPI()

sim = SimWrapper()
SESSION = {
    "base_prompt": None,
    "initial_command": None,
    "history": []
}

async def lifespan(app: FastAPI):
    prompt_path = Path('/home/jingyang/robosuite/myCode/my_planning_app/prompts/base.json')
    try:
        base_prompt = load_default_prompt(prompt_path)
        SESSION["base_prompt"] = base_prompt
        raw_objects = sim.get_current_state()
        scene_objects = [ObjectSpec(**obj) for obj in raw_objects.values()]
        expected_objects = base_prompt.environment.get("objects", [])
        print("🔍 Checking scene consistency at startup...")
        check_scene_consistency(scene_objects, expected_objects, threshold=0.001)
    except Exception as e:
        print(f"⚠️ Startup error: {e}")
    yield

app = FastAPI(lifespan=lifespan)

@app.post("/init_session")
def init_session(payload: PromptInput):
    SESSION["base_prompt"] = payload
    SESSION["initial_command"] = None
    SESSION["history"] = []
    return {"status": "Session initialized"}

@app.post("/chat_step")
def chat_step(command: str, mode: str = None):
    payload = SESSION["base_prompt"]
    if SESSION["initial_command"] is None:
        mode = "override"
        SESSION["initial_command"] = command
    else:
        mode = mode or "chain"

    SESSION["history"].append((mode, command))

    raw_objects = sim.get_current_state()
    object_specs = [ObjectSpec(**obj) for obj in raw_objects.values()]
    payload.environment["objects"] = object_specs

    try:
        check_scene_consistency(object_specs, payload.environment["objects"], threshold=0.05)
    except ValueError as e:
        return {"error": "Scene mismatch", "details": str(e)}
    
    prompt = construct_prompt(payload, command, mode, SESSION["initial_command"])
    try:
        result = call_llm(prompt)
    except Exception as e:
        return {"error": f"LLM failed: {e}"}

    sim.execute_plan(result["symbolic_plan"])
    return {
        "mode": mode,
        "command": command,
        "explanation": result["explanation"],
        "symbolic_plan": result["symbolic_plan"],
        "current_objects": raw_objects
    }

@app.post("/reset_scene_and_robot")
def reset_scene_and_robot():
    global sim
    try:
        sim.idle_and_close()
        sim = SimWrapper()
        SESSION["initial_command"] = None
        SESSION["history"] = []
        return {"status": "Scene and robot reset"}
    except Exception as e:
        return {"error": str(e)}

@app.post("/reset_robot_only")
def reset_robot_only():
    sim.reset_robot()
    SESSION["initial_command"] = None
    return {"status": "Robot reset, scene preserved"}

@app.get("/")
def read_root():
    return {"message": "Interactive Planning API running. Use /init_session then /chat_step."}
