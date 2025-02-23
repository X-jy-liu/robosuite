import numpy as np
import time

from robosuite import make
from robosuite.controllers import controller_factory, load_part_controller_config

# 步骤1：加载OSC_POSE控制器配置
osc_config = load_part_controller_config(
    custom_fpath="/home/jingyang/robosuite/robosuite/controllers/config/default/parts/osc_pose.json",
    default_controller="OSC_POSE"
)

# 步骤2：创建控制器实例
controller = {'type': 'BASIC', 
              'body_parts': 
                {'right':
                  {'type': 'OSC_POSE', 
                  'input_max': 1, 
                  'input_min': -1, 
                  'output_max': [0.05, 0.05, 0.05, 0.5, 0.5, 0.5], 
                  'output_min': [-0.05, -0.05, -0.05, -0.5, -0.5, -0.5], 
                  'kp': 150, 'damping_ratio': 1, 
                  'impedance_mode': 'fixed', 
                  'kp_limits': [0, 300], 
                  'damping_ratio_limits': [0, 10], 
                  'position_limits': None, 
                  'orientation_limits': None, 
                  'uncouple_pos_ori': True, 
                  'input_type': 'delta', 
                  'input_ref_frame': 'base', 
                  'interpolation': None, 
                  'ramp_ratio': 0.2, 
                  'gripper': {'type': 'GRIP'}}}}


# 步骤3：初始化环境
env = make(
    env_name="Lift",
    robots="Panda",
    controller_configs=controller,
    has_renderer=True,
)

# 步骤4：控制逻辑
target_pos = np.array([-0.1, 0.1, 1.5])

obs = env.reset()

for _ in range(1000):

    # # random actions testing    
    # action = np.random.randn(*env.action_spec[0].shape) * 0.1
    # obs, reward, done, info = env.step(action)  # take action in the environment
    # env.render()  # render on display
    # 获取当前状态
    current_pos = obs["robot0_eef_pos"]
    error = target_pos - current_pos
    
    # 生成控制指令（格式：[dx, dy, dz, droll, dpitch, dyaw, gripper]）
    action = np.concatenate([error * 3, [0, 0, 0], [-1]])
    
    # 执行动作
    obs, _, _, _ = env.step(action)
    env.render()
    
    # 终止条件
    if np.linalg.norm(error) < 0.01:
        print(f"Success! Reached position {target_pos} from {initial_pos}")
        time.sleep(2)
        break
env.close()
