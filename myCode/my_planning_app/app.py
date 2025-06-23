from fastapi import FastAPI
from prompt_engine.prompt_loader import load_default_prompt
from prompt_engine.prompt_builder import construct_prompt
from prompt_engine.models import ObjectSpec, ChatRequest, InitSessionRequest
from prompt_engine.llm_interface import call_llm
from prompt_engine.sim_wrapper import SimWrapper
from prompt_engine.scene_checker import check_scene_consistency
from pathlib import Path
from contextlib import asynccontextmanager
from prompt_engine.utils import log_task_summary
from prompt_engine.scene_generator import SceneGenerator
from prompt_engine.dots_generator import DotGenerator
import time
from datetime import datetime
import random
from fastapi.responses import HTMLResponse
import json


app = FastAPI()

SESSION = {
    "base_prompt": None,
    "initial_command": None,
    "scene_config_path": None,
    "reference_dots": None,
    "history": [],
    "current_task_logs": []
}

@asynccontextmanager
async def lifespan(app: FastAPI):
    # No scene generation or prompt loading here anymore
    print("🚀 Server starting...")
    yield
    print("🛑 Server shutting down...")

app = FastAPI(lifespan=lifespan)

@app.post("/init_session")
def init_session(req: InitSessionRequest):
    try:
        HOME_DIR = Path.home()
        save_path = HOME_DIR / "robosuite" / "myCode" / "my_planning_app" / "prompts" / "env_and_func.json"
        SESSION["scene_config_path"] = str(save_path)

        if req.regenerate_scene or not save_path.exists():
            # ✅ Generate a new scene
            generator = SceneGenerator()
            scene = generator.generate_scene(num_objects=5, seed=random.randint(0, 10000))
            generator.save_to_json(scene, save_path)
            print("🆕 New scene generated and saved.")
        else:
            scene = load_default_prompt(save_path)
            print("♻️ Reusing existing scene.")

        global sim
        sim = SimWrapper(scene_config_path=save_path)

        # Dots
        dots_generator = DotGenerator(save_path)
        valid_dots = dots_generator.generate_valid_dots(num_dots=5, clearance=0.08, seed=random.randint(0, 10000))
        # save the valid dots in .json format
        dots_output_path = HOME_DIR / "robosuite" / "myCode" / "my_planning_app" / "prompts" / "generated_dots.json"
        dots_generator.save_dots_to_json(valid_dots, dots_output_path)

        SESSION["reference_dots"] = valid_dots
        # Prompt
        base_prompt = load_default_prompt(save_path)
        SESSION["base_prompt"] = base_prompt

        # Check scene consistency
        raw_objects = sim.get_current_state()
        scene_objects = [ObjectSpec(**obj) for obj in raw_objects.values()]
        expected_objects = base_prompt.environment.get("objects", [])
        check_scene_consistency(current_objs=scene_objects, expected_objs=expected_objects, threshold=0.05)

        # Reset session metadata
        SESSION["initial_command"] = None
        SESSION["history"] = []
        SESSION["current_task_logs"] = []

        return {
            "status": "Session initialized",
            "regenerated": req.regenerate_scene,
            "scene": scene,
            "reference_dots": valid_dots

        }

    except Exception as e:
        return {"error": f"Session initialization failed: {e}"}

@app.get("/show_scene", response_class=HTMLResponse)
def show_scene():
    try:
        raw_objects = SESSION["base_prompt"].environment.get("objects", []) if SESSION["base_prompt"] else []
        objects = [obj if isinstance(obj, dict) else obj.__dict__ for obj in raw_objects]
        dots = SESSION["reference_dots"] if SESSION["reference_dots"] else []
        print(dots)

        return f"""
        <html>
        <head>
            <title>Scene Visualization</title>
            <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
        </head>
        <body>
            <h2>Scene Objects and Reference Dots</h2>
            <div id="scenePlot" style="width:90vw;height:80vh;"></div>
            <script>
                const objects = {json.dumps(objects)};
                const dots = {json.dumps(dots)};

                const objTrace = {{
                    x: objects.map(o => o.position[0]),
                    y: objects.map(o => o.position[1]),
                    text: objects.map(o => o.shape + " (" + o.color + ")"),
                    mode: 'markers+text',
                    type: 'scatter',
                    name: 'Objects',
                    marker: {{ size: 15, color: 'blue' }},
                    textposition: 'top center'
                }};

                const dotTrace = {{
                    x: dots.map(d => d[0]),
                    y: dots.map(d => d[1]),
                    mode: 'markers',
                    type: 'scatter',
                    name: 'Reference Dots',
                    marker: {{ size: 10, color: 'red', symbol: 'cross' }}
                }};

                const layout = {{
                    title: 'Top-down Scene View',
                    xaxis: {{
                        title: 'X Position',
                        range: [-0.4, 0.4],
                        constrain: 'domain',
                    }},
                    yaxis: {{
                        title: 'Y Position',
                        range: [-0.4, 0.4],
                        scaleanchor: "x"
                    }},
                    showlegend: true
                }};

                Plotly.newPlot('scenePlot', [objTrace, dotTrace], layout);
            </script>
        </body>
        </html>
        """
    except Exception as e:
        return HTMLResponse(f"<h2>Error visualizing scene: {e}</h2>")

@app.post("/chat_step")
def chat_step(request: ChatRequest):
    # Check if the simulation environment is initialized
    if "sim" not in globals() or sim is None:
        return {"error": "Simulation environment is not initialized. Call /init_session first."}
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
        # loggin the interpertation time
        start_time = time.time()
        result = call_llm(prompt)
        llm_time = time.time() - start_time
    except Exception as e:
        return {"error": f"LLM or prompt failed: {e}"}

    sim.execute_plan(result["symbolic_plan"])
    curr_objects = sim.get_current_state()

    step_log = {
            "timestamp": datetime.now().isoformat(),
            "mode": mode,
            "task_type": request.task_type,
            "task_command": request.command,
            "symbolic_plan": result["symbolic_plan"],
            "explanation": result["explanation"],
            "llm_interpretation_time_sec": round(llm_time, 3),
            "start_env": prev_objects,
            "end_env": curr_objects
        }
    
    SESSION["current_task_logs"].append(step_log)
    
    return step_log

@app.post("/save_logs")
def save_logs():
    try:
        if not SESSION["current_task_logs"]:
            return {"status": "Nothing to save. No active task steps."}
        log_task_summary(SESSION)
        return {"status": "Current task logs saved successfully."}
    except Exception as e:
        return {"error": str(e)}
    
# @app.post("/reset_scene")
# def reset_scene():
#     try:
#         if "scene_config_path" not in SESSION:
#             return {"error": "No scene found to reset. Please call /init_session first."}

#         # Soft reset the scene without restarting SimWrapper
#         sim.reset_scene()

#         # Re-generate reference dots
#         scene_config_path = Path(SESSION["scene_config_path"])
#         dots_generator = DotGenerator(scene_config_path)
#         valid_dots = dots_generator.generate_valid_dots(num_dots=5, clearance=0.08, seed=random.randint(0, 10000))
#         SESSION["reference_dots"] = valid_dots

#         # Reload prompt and consistency check
#         base_prompt = load_default_prompt(scene_config_path)
#         SESSION["base_prompt"] = base_prompt
#         raw_objects = sim.get_current_state()
#         scene_objects = [ObjectSpec(**obj) for obj in raw_objects.values()]
#         expected_objects = base_prompt.environment.get("objects", [])
#         print(("debug: expected_objects", expected_objects))
#         check_scene_consistency(current_objs=scene_objects, expected_objs=expected_objects, threshold=0.05)

#         # Reset session state
#         SESSION["initial_command"] = None
#         SESSION["history"] = []
#         SESSION["current_task_logs"] = []

#         return {"status": "Scene reset to initial configuration"}
#     except Exception as e:
#         return {"error": f"Scene reset failed: {e}"}

# @app.post("/reset_scene_and_robot")
# def reset_scene_and_robot():
#     global sim
#     try:
#         log_task_summary(SESSION)
#         sim.idle_and_close()
#         sim = SimWrapper()
#         SESSION["initial_command"] = None
#         SESSION["history"] = []
#         return {"status": "Scene and robot reset"}
#     except Exception as e:
#         return {"error": str(e)}

# @app.post("/reset_robot_only")
# def reset_robot_only():
#     sim.reset_robot()
#     SESSION["initial_command"] = None
#     return {"status": "Robot reset, scene preserved"}

@app.get("/")
def read_root():
    return {"message": "Interactive Planning API running. Use /init_session then /chat_step."}
