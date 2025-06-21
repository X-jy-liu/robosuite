# prompt_engine/scene_checker.py
import numpy as np
from prompt_engine.models import ObjectSpec

def check_scene_consistency(current_objs: list[ObjectSpec], expected_objs: list[ObjectSpec], threshold=0.01):
    mismatches = []
    expected_dict = {obj.name: obj for obj in expected_objs}
    half_height = 0.025
    for obj in current_objs:
        if obj.name in expected_dict:
            expected = expected_dict[obj.name]
            cur_pos = np.array(obj.position[:3])

            table_height = 0.8
            exp_pos_raw = expected.position
            if len(exp_pos_raw) == 2:
                z_val = table_height + half_height
                exp_pos = np.array([*exp_pos_raw, z_val])
            else:
                exp_pos = np.array(exp_pos_raw[:3])

            dist = np.linalg.norm(exp_pos - cur_pos)
            if dist > threshold:
                mismatches.append((obj.name, dist, cur_pos.tolist(), exp_pos.tolist()))

    if mismatches:
        details = "\n".join([
            f"{name}: distance={dist:.3f}, current={cur}, expected={exp}"
            for name, dist, cur, exp in mismatches
        ])
        raise ValueError(f"🚨 Scene mismatch detected:\n{details}")
    else:
        print("✅ Scene consistency check passed.")
