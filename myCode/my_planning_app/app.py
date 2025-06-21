from fastapi import FastAPI
from prompt_engine.prompt_loader import load_default_prompt
from prompt_engine.prompt_builder import construct_prompt
from prompt_engine.models import PromptInput, ObjectSpec, ChatRequest
from prompt_engine.llm_interface import call_llm
from prompt_engine.sim_wrapper import SimWrapper
from prompt_engine.scene_checker import check_scene_consistency
from pathlib import Path
from contextlib import asynccontextmanager
import json

app = FastAPI()

sim = SimWrapper()
SESSION = {
    "base_prompt": None,
    "initial_command": None,
    "history": []
}

@asynccontextmanager
async def lifespan(app: FastAPI):
    HOME_DIR = Path.home()
    base_prompt_path = HOME_DIR / "robosuite" / "myCode" / "my_planning_app" / "prompts" / "env_and_func.json"
    if not base_prompt_path.exists():
        raise FileNotFoundError(f"Default prompt file not found at {base_prompt_path}")
    try:
        base_prompt = load_default_prompt(base_prompt_path)
        SESSION["base_prompt"] = base_prompt
        raw_objects = sim.get_current_state()
        scene_objects = [ObjectSpec(**obj) for obj in raw_objects.values()]
        expected_objects = base_prompt.environment.get("objects", [])
        print("🔍 Checking scene consistency at startup...")
        check_scene_consistency(
            current_objs=scene_objects,
            expected_objs=expected_objects, # loaded from prompt
            threshold=0.05
        )
    except Exception as e:
        print("🔍 base_prompt =", base_prompt)
        print("🔍 type(base_prompt) =", type(base_prompt))
        print(f"⚠️ Startup error: {e}")

    yield

app = FastAPI(lifespan=lifespan)

@app.post("/init_session")
def init_session():
    SESSION["initial_command"] = None
    SESSION["history"] = []
    return {"status": "Session initialized"}

@app.post("/chat_step")
def chat_step(request: ChatRequest):
    if SESSION["initial_command"] is None:
        SESSION["initial_command"] = request.command
        mode = "override"
    else:
        mode = request.mode or "chain"

    SESSION["history"].append((mode, request.command))

    prev_objects = sim.get_current_state()

    try:
        prompt = construct_prompt(
            command=request.command,
            task_type=request.task_type,
            mode=mode,
            initial_command=SESSION["initial_command"]
        )
        result = call_llm(prompt)
    except Exception as e:
        return {"error": f"LLM or prompt failed: {e}"}

    sim.execute_plan(result["symbolic_plan"])
    curr_objects = sim.get_current_state()

    return {
        "mode": mode,
        "command": request.command,
        "explanation": result["explanation"],
        "symbolic_plan": result["symbolic_plan"],
        "prev_objects": prev_objects,
        "curr_objects": curr_objects,
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
