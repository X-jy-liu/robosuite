from fastapi import FastAPI
from prompt_engine.prompt_loader import load_default_prompt, load_reference_dots
from prompt_engine.prompt_builder import construct_prompt
from prompt_engine.trajectory_prompt_builder import construct_trajectory_prompt
from prompt_engine.models import ObjectSpec, ChatRequest, InitSessionRequest
from prompt_engine.llm_interface import call_llm
from prompt_engine.sim_wrapper import SimWrapper
from prompt_engine.scene_checker import check_scene_consistency
from pathlib import Path
from contextlib import asynccontextmanager
from prompt_engine.utils import log_task_summary
from prompt_engine.scene_generator import SceneGenerator, ThreeObjGenerator
from prompt_engine.dots_generator import DotGenerator
import time
from datetime import datetime
import random
from fastapi.responses import HTMLResponse
from trajectory_planning.json_loader import load_objects_from_json, load_dots_from_json
from trajectory_planning.rpm_generator import RPMGenerator
from trajectory_planning.dijkstra_path_from_points import dijkstra_path_from_points,build_symbolic_plan
import json
import matplotlib.pyplot as plt
import io
import base64

app = FastAPI()

SESSION = {
    "base_prompt": None,
    "initial_command": None,
    "task_type": None,
    "scene_dir": None,
    "reference_dots": None,
    "obj_specs": None,
    "pnt_specs": None,
    "cmd_history": [],
    "current_task_logs": []
}

@asynccontextmanager
async def lifespan(app: FastAPI):
    # No scene generation or prompt loading here anymore
    print("API Loaded")
    print("🚀 Server starting...")
    yield
    print("🛑 Server shutting down...")

app = FastAPI(lifespan=lifespan)

@app.post("/init_session")
def init_session(req: InitSessionRequest):
    try:
        HOME_DIR = Path.home()
        if req.scene_number is not None:
            print(f"Initializing session for predefined scene {req.scene_number} ...")
            scene_dir = HOME_DIR / "robosuite" / "myCode" / "my_planning_app" / "logs" / f"scene_{req.scene_number:02d}"
        else:
            print("Initializing session from the prompt directory ...")
            scene_dir = HOME_DIR / "robosuite" / "myCode" / "my_planning_app" / "prompts"
        SESSION["scene_dir"] = scene_dir
        if req.phase is None:
            raise ValueError("Phase must be provided for session initialization.")
        # assign the dot path
        dots_path = scene_dir / "generated_dots.json"

        # assign the scene path based on the phase
        SESSION["phase"] = req.phase
        if SESSION["phase"] == 1:
            print("Phase 1: loading fully described scene from JSON ...")
            scene_path = scene_dir / "env_and_func.json"
        elif SESSION["phase"] == 2:
            print("Phase 2: Rendering scene and running perception ...")
            gt_scene_path = scene_dir / "env_and_func.json"
            # instantiate the simulation environment
            #  1. initialize the class with the gt scene path
            #  2. save the scene as a rendered image
            render_sim = SimWrapper(scene_config_path=gt_scene_path, robot_offset=True)
            render_sim.scene_render(scene_dir)
            #  3. percept objects and save the rendered image
            render_sim.yolo_perception_and_save(
                gt_env_and_func_path= gt_scene_path,
                rendered_img_path = scene_dir / "env_and_func_rendered.png",
                checkpoint_path= HOME_DIR / "robosuite" / "myCode" / "yolo_perception" / "runs" / "detect" / "mini_train_debug" / "weights" / "best.pt"
                )
            # hardcode the file name for the predicted scene path
            scene_path = scene_dir / "env_and_func_predicted.json"
        elif SESSION["phase"] == 3:
            pass
        else:
            raise ValueError(f"Invalid phase: {SESSION['phase']}. Must be 1, 2, or 3.")

        # === Scene Generation ===
        if req.ambiguous_effects:
            print("Generating scene with 3 objects for ambiguous effects experiment ...")
            generator = ThreeObjGenerator(obj_bounds=[-0.22, 0.22], object_size=0.05)
            scene = generator.generate_three_objects_with_ratio(ratio=2.0, seed=random.randint(0, 10000))
            generator.save_to_json(scene, scene_path)
            print(f"Ambiguous effects scene generated and saved at {scene_path}.")
        else:
            if req.regenerate_scene or not scene_path.exists():
                print("Generating new scene ...")
                generator = SceneGenerator(obj_bounds=[-0.22, 0.22], min_distance=0.11, object_size=0.05)
                scene = generator.generate_scene(num_objects=5, seed=random.randint(0, 10000))
                generator.save_to_json(scene, scene_path)
                print(f"New scene generated and saved at {scene_path}.")
            else:
                print(f"Reusing existing scene ...")
                print(f"Loaded from {scene_path}...")

            # === Dots Generation ===
            dots_generator = DotGenerator(env_and_func_path = scene_path, dots_bounds = [-0.25, 0.25])
            if req.regenerate_dots or not dots_path.exists():
                print("Generating reference dots ...")
                valid_dots = dots_generator.generate_valid_dots(num_dots=5, 
                                                                buffer=0.05, 
                                                                min_dot_distance=0.05, 
                                                                seed=random.randint(0, 10000))
                dots_generator.save_dots_to_json(valid_dots, dots_path)
                print("New dots generated and saved.")
            else:
                print("Reusing existing dots ...")
                valid_dots = dots_generator.load_dots_from_json(dots_path)
                print(f"Reference dots loaded from {dots_path}.")
            SESSION["reference_dots"] = valid_dots

            # === Generate RPM ===
            print("Generating roadmap PRM ...")
            obj_specs = load_objects_from_json(scene_path)
            pnt_specs = load_dots_from_json(dots_path)
            rpm_generator = RPMGenerator(obj_specs, pnt_specs, roadmap_buffer=0.05, max_rpm=200)
            graph, nodes = rpm_generator.build(roadmap_bounds=[-0.28, 0.28])
            print("Roadmap PRM generated successfully.")
            print(f"Number of nodes in graph: {len(nodes)}")
            print(f"Number of edges in graph: {len(graph.edges)}")
            
            SESSION["roadmap_graph"] = graph
            SESSION["roadmap_nodes"] = nodes
            SESSION["obj_specs"] = obj_specs
            SESSION["pnt_specs"] = pnt_specs
        
        # ambiguous effects experiment won't have a roadmap, so we skip this part
        # === Sim & Prompt Setup ===
        print("Initializing simulation environment...")
        global sim
        sim = SimWrapper(scene_config_path=scene_path, record_video=req.record_video, video_path=scene_dir / "execution_video.mp4")
        base_prompt = load_default_prompt(scene_path)

        SESSION["base_prompt"] = base_prompt
        dots_description = load_reference_dots(dots_path)
        print("Base prompt and reference dots loaded successfully.")
        # === Check Consistency ===
        raw_objects = sim.get_current_state()
        scene_objects = [ObjectSpec(**obj) for obj in raw_objects.values()]
        expected_objects = base_prompt.environment.get("objects", [])
        check_scene_consistency(current_objs=scene_objects, expected_objs=expected_objects, threshold=0.05)

        # === Reset Metadata ===
        SESSION["initial_command"] = None
        SESSION["cmd_history"] = []
        SESSION["current_task_logs"] = []

        return {
            "status": "Session initialized",
            "regenerated_scene": req.regenerate_scene,
            "regenerated_dots": req.regenerate_dots,
            "env_and_func": base_prompt,
            "reference_dots": dots_description
        }

    except Exception as e:
        return {"error": f"Session initialization failed: {e}"}

@app.get("/show_scene", response_class=HTMLResponse)
def show_scene():
    try:
        raw_objects = SESSION["base_prompt"].environment.get("objects", []) if SESSION["base_prompt"] else []
        # load the objects from the session
        objects = [
            {
                "name": obj.name,
                "shape": obj.shape,
                "color": obj.color,
                "position": obj.position,
                "size": obj.size
            }
            for obj in raw_objects
        ]
        # Load reference dots from session
        dots = SESSION.get("reference_dots", [])

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
                    x: objects.map(o => o.position[1]),
                    y: objects.map(o => o.position[0]),
                    text: objects.map(o => o.name + ": " + o.shape + " (" + o.color + ")"),
                    mode: 'markers+text',
                    type: 'scatter',
                    name: 'Objects',
                    marker: {{
                        size: 15,
                        color: objects.map(o => o.color),
                        symbol: objects.map(o => o.shape === "cube" ? "square" : "circle")
                    }},
                    textposition: 'top center'
                }};

                const dotTrace = {{
                    x: dots.map(d => d[1]),
                    y: dots.map(d => d[0]),
                    text: dots.map((_, i) => "Point " + (i + 1)),
                    mode: 'markers+text',
                    type: 'scatter',
                    name: 'Reference Dots',
                    marker: {{
                        size: 10,
                        color: 'yellow',
                        symbol: 'x'
                    }},
                    textposition: 'bottom center'
                }};

                const layout = {{
                    title: 'Top-down Scene View',
                    xaxis: {{
                        title: 'X Position',
                        range: [-0.4, 0.4],
                        constrain: 'domain'
                    }},
                    yaxis: {{
                        title: 'Y Position',
                        range: [0.4, -0.4],
                        scaleanchor: 'x'
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
    
@app.get("/show_rpm", response_class=HTMLResponse)
def show_rpm():
    try:
        rpm = RPMGenerator(SESSION["obj_specs"], SESSION["pnt_specs"], roadmap_buffer=0.05, max_rpm=100)
        # Generate plot as image
        fig, ax = plt.subplots(figsize=(6, 6))

        rpm.visualize_prm(
            SESSION.get("roadmap_graph"), 
            SESSION.get("roadmap_nodes"), 
            SESSION["obj_specs"], 
            SESSION["pnt_specs"])
        
        buf = io.BytesIO()
        plt.savefig(buf, format='png')
        plt.close(fig)
        buf.seek(0)
        img_base64 = base64.b64encode(buf.getvalue()).decode('utf-8')
        html_content = f"""
        <html>
        <head><title>PRM Visualization</title></head>
        <body>
        <h2>Roadmap PRM Visualization</h2>
        <img src="data:image/png;base64,{img_base64}" />
        </body>
        </html>
        """
        return HTMLResponse(content=html_content)
    except Exception as e:
        return HTMLResponse(f"<h2>Error generating PRM: {e}</h2>")

@app.post("/chat_step")
def chat_step(request: ChatRequest):
    if "sim" not in globals() or sim is None:
        return {"error": "Simulation environment is not initialized. Call /init_session first."}

    commands = request.commands if isinstance(request.commands, list) else [request.commands]
    all_logs = []
    print(f"Commands received: \n   {commands}")

    for i, cmd in enumerate(commands):
        mode = "override" if (i == 0 and SESSION["initial_command"] is None) else "chain"
        print(f"Executing command {i+1}/{len(commands)}: {cmd} (mode: {mode}) ... ")
        if SESSION["initial_command"] is None:
            SESSION["initial_command"] = cmd

        SESSION["cmd_history"].append((mode, cmd))
        SESSION["task_type"] = request.task_type
        init_obj = sim.get_current_state()

        try:
            if request.task_type != "trajectory":
                prompt = construct_prompt(cmd, request.task_type, mode, SESSION["scene_dir"], SESSION["initial_command"])
                # print("---------- prompt start ----------\n", prompt, "\n---------- prompt end ----------")
                print(f"🧠 Calling LLM for the command ......\n\"{cmd}\"")
                start_time = time.time()
                result = call_llm(prompt, task_type=request.task_type)
                llm_time = time.time() - start_time
                symbolic_plan = result["symbolic_plan"]
                # print("---------- symbolic plan ----------\n", symbolic_plan)
            else:
                prompt = construct_trajectory_prompt(cmd, request.task_type, mode, SESSION["scene_dir"])
                # print("---------- prompt start ----------\n", prompt, "\n---------- prompt end ----------")
                print(f"🧠 Calling LLM for the command ......\n\"{cmd}\"")
                start_time = time.time()
                result = call_llm(prompt, task_type=request.task_type)
                llm_time = time.time() - start_time
                trajectory_points = result.get("trajectory_points")
                if not isinstance(trajectory_points, list) or not all(isinstance(pt, list) and len(pt) == 2 for pt in trajectory_points):
                    return {"error": f"Invalid trajectory points format: {trajectory_points}"}
                _, waypoints = dijkstra_path_from_points(trajectory_points, SESSION["roadmap_graph"], SESSION["roadmap_nodes"])
                symbolic_plan = build_symbolic_plan(waypoints)
                # print("---------- symbolic plan ----------\n", symbolic_plan)
        except Exception as e:
            return {"error": f"LLM or prompt failed: {e}"}

        obj_pos_history = sim.execute_plan(symbolic_plan)

        cmd_step_log = {
            "timestamp": datetime.now().isoformat(),
            "mode": mode,
            "task_type": request.task_type,
            "task_command": cmd,
            "symbolic_plan": symbolic_plan,
            "explanation": result["explanation"],
            "llm_interpretation_time_sec": round(llm_time, 3),
            "init_obj": init_obj,
            "obj_pos_history": obj_pos_history
        }

        all_logs.append(cmd_step_log)

    SESSION["current_task_logs"] = all_logs

    api_repsonse = {
        "mode": mode,
        "task_type": request.task_type,
        "num_commands": len(commands),
        "status_message": "cmd steps executed successfully and logged in cache. Use /save_logs to save them."
    }
    
    return api_repsonse

@app.post("/save_logs")
def save_logs():
    try:
        if not SESSION["current_task_logs"]:
            return {"status": "Nothing to save. No active task steps."}
        log_task_summary(SESSION, save_dir=SESSION["scene_dir"], phase=SESSION["phase"])
        return {"status": "Current task logs saved successfully."}
    except Exception as e:
        return {"error": str(e)}
    
@app.post("/run_full_experiment")
def run_full_experiment(params: dict):
    try:
        print("Received experiment params:", params)

        init_result = init_session(InitSessionRequest(
            regenerate_scene=params.get("regenerate_scene", False),
            regenerate_dots=params.get("regenerate_dots", False),
            scene_number=params.get("scene_number", None),
            phase=params.get("phase", None)
        ))
        if "error" in init_result:
            return {"init_error": init_result}

        chat_result = chat_step(ChatRequest(
            commands=params["commands"],
            task_type=params["task_type"],
            mode=params.get("mode", "override")
        ))
        if "error" in chat_result:
            return {"chat_error": chat_result}

        save_result = save_logs()
        return {
            "init_result": init_result,
            "chat_result": chat_result,
            "save_result": save_result
        }
    except Exception as e:
        import traceback
        print("Exception in run_full_experiment:", traceback.format_exc())
        return {"error": f"Exception occurred: {str(e)}"}
    
@app.post("/run_ambiguous_experiment")
def run_ambiguous_experiment(params: dict):
    """
    Run an ambiguous experiment using a 2-step interaction: initial + follow-up command.
    Params must include:
    {
        "commands": list[str]
        "task_type": str,
        "regenerate_scene": bool,
        "regenerate_dots": bool
        "ambiguous_effects": bool
    }
    """
    try:
        print("Running ambiguous experiment with params:", params)
        
        # === 1. Init Session ===
        init_result = init_session(InitSessionRequest(
            regenerate_scene=params.get("regenerate_scene", False),
            regenerate_dots=params.get("regenerate_dots", False),
            ambiguous_effects=params.get("ambiguous_effects", False)
        ))
        if "error" in init_result:
            return {"init_error": init_result}

        # === 2. Merge and Run Commands ===
        commands = params.get("commands", [])
        chat_result = chat_step(ChatRequest(
            commands=commands,
            task_type=params["task_type"],
            mode="override"  # only applies to the first command; chat_step handles the rest as 'chain'
        ))
        if "error" in chat_result:
            return {"chat_error": chat_result}

        # === 3. Save Logs ===
        save_result = save_logs()

        return {
            "init_result": init_result,
            "chat_result": chat_result,
            "save_result": save_result
        }

    except Exception as e:
        import traceback
        print("Exception in run_ambiguous_experiment:", traceback.format_exc())
        return {"error": f"Exception occurred: {str(e)}"}

@app.get("/")
def read_root():
    return {"message": "Interactive Planning API running. Use /init_session then /chat_step."}