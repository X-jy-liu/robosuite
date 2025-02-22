import numpy as np
import robosuite as suite

# Create environment instance
env = suite.make(
    env_name="Lift",  # Task: Lifting an object
    robots="Panda",   # Robot: Panda arm
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
)

# Reset the environment
env.reset()

# === Skill Set Functions === #

def move_to(env, target_pos, duration=50):
    """
    Moves the robot's end-effector to a target position smoothly.

    :param env: robosuite environment instance
    :param target_pos: Target position in 3D space (x, y, z)
    :param duration: Number of simulation steps for the movement
    """
    for _ in range(duration):
        action = np.zeros(env.robots[0].action_dim)  # Initialize action
        current_pos = env.robots[0].controller.ee_pos  # Get current end-effector position
        
        # Calculate delta movement
        delta = (np.array(target_pos) - current_pos) * 0.1  # Scale movement for smoothness
        action[:3] = delta  # Move end-effector positionally
        
        env.step(action)
        env.render()

def grab(env, object_name):
    """
    Grasps an object by moving to its position and closing the gripper.
    
    :param env: robosuite environment instance
    :param object_name: Name of the object to grasp
    """
    obj_pos = env.sim.data.get_site_xpos(object_name)  # Get object position
    move_to(env, obj_pos)  # Move above the object
    move_to(env, obj_pos + np.array([0, 0, 0.1]))  # Lower slightly above

    # Close the gripper
    for _ in range(20):
        action = np.zeros(env.robots[0].action_dim)
        action[-1] = -1  # Close gripper
        env.step(action)
        env.render()

def release(env):
    """
    Opens the gripper to release an object.
    
    :param env: robosuite environment instance
    """
    for _ in range(20):
        action = np.zeros(env.robots[0].action_dim)
        action[-1] = 1  # Open gripper
        env.step(action)
        env.render()

def push(env, object_name, direction, distance=0.1):
    """
    Pushes an object in the specified direction for a given distance.
    
    :param env: robosuite environment instance
    :param object_name: Object to push
    :param direction: A tuple (dx, dy) indicating push direction
    :param distance: Distance to push the object
    """
    obj_pos = env.sim.data.get_site_xpos(object_name)
    start_pos = np.array(obj_pos)
    end_pos = start_pos + np.array([direction[0] * distance, direction[1] * distance, 0])

    move_to(env, start_pos)  # Move to object
    move_to(env, end_pos)    # Push object along the direction

def place(env, object_name, target_pos):
    """
    Moves an object to a target position and releases it.
    
    :param env: robosuite environment instance
    :param object_name: Object to move
    :param target_pos: (x, y, z) target position
    """
    grab(env, object_name)
    move_to(env, target_pos)
    release(env)
