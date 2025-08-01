import ast
import matplotlib.pyplot as plt
from pathlib import Path

# Symbolic plan groups
basic_plans = [
    "[[\"move\", \"obj2\"], [\"grip_and_pickup\", \"obj2\"]]",
    "[[\"move\", \"obj3\"], [\"grip_and_pickup\", \"obj3\"], [\"move\", [0.1, 0.1, 0.825]], [\"gripper_open\"]]",
    "[[\"move\", \"obj2\"], [\"grip_and_pickup\", \"obj2\"], [\"move\", \"obj0\"], [\"gripper_open\"]]"
]

ambiguous_plans = [
    "[[\"move\", \"obj3\"], [\"grip_and_pickup\", \"obj3\"], [\"move\", [0.2, -0.14, 0.8]], [\"gripper_open\"]]",
    "[[\"move\", \"obj0\"], [\"grip_and_pickup\", \"obj0\"], [\"move\", [-0.2, 0.18, 0.8]], [\"gripper_open\"]]",
    "[[\"move\", \"obj4\"], [\"grip_and_pickup\", \"obj4\"], [\"move\", [0.3, -0.3, 0.8]], [\"gripper_open\"]]"
]

trajectory_plans = [
    "[[\"gripper_move\", [0.2,-0.2,0.875]],[\"gripper_open\"],[\"gripper_move\", [0.2,-0.2,0.825]],[\"gripper_close\"],[\"gripper_move\", [0.2,-0.2,0.835]],[\"gripper_move\", [0.1,-0.1,0.835]],[\"gripper_move\", [-0.1,-0.1, 0.835]],[\"gripper_move\", [0.05, 0.2, 0.835]],[\"gripper_open\"]]",
    "[[\"gripper_move\", [0.15, 0.15, 0.875]],[\"gripper_open\"],[\"gripper_move\", [0.15, 0.15, 0.825]],[\"gripper_close\"],[\"gripper_move\", [0.15, 0.15, 0.835]],[\"gripper_move\", [0.1, -0.1, 0.835]],[\"gripper_move\", [-0.29, 0.0, 0.835]],[\"gripper_open\"]]",
    "[[\"gripper_move\", [0.25, 0.25, 0.875]],[\"gripper_open\"],[\"gripper_move\", [0.25, 0.25, 0.825]],[\"gripper_close\"],[\"gripper_move\", [0.25, 0.25, 0.835]],[\"gripper_move\", [0.15, -0.15, 0.835]],[\"gripper_move\", [0.05, -0.25, 0.835]],[\"gripper_move\", [-0.3, 0.1, 0.835]],[\"gripper_open\"]]"
]

# Token counting function
def count_tokens(action):
    if isinstance(action, list):
        return sum(count_tokens(item) for item in action)
    elif isinstance(action, (str, int, float)):
        return 1
    else:
        return 0

# Get token counts
def get_token_counts(plans):
    counts = []
    for plan_str in plans:
        plan = ast.literal_eval(plan_str)
        total_tokens = sum(count_tokens(action) for action in plan)
        print(f"Plan: {plan_str} | Token Count: {total_tokens}")
        counts.append(total_tokens)
    return counts

# Count tokens
basic_tokens = get_token_counts(basic_plans)
ambiguous_tokens = get_token_counts(ambiguous_plans)
trajectory_tokens = get_token_counts(trajectory_plans)

mean_basic = sum(basic_tokens) / len(basic_tokens) if basic_tokens else 0
mean_ambiguous = sum(ambiguous_tokens) / len(ambiguous_tokens) if ambiguous_tokens else 0
mean_trajectory = sum(trajectory_tokens) / len(trajectory_tokens) if trajectory_tokens else 0
print(f"Basic Plans Token Mean Counts: {mean_basic}")
print(f"Ambiguous Plans Token Mean Counts: {mean_ambiguous}")
print(f"Trajectory Plans Token Mean Counts: {mean_trajectory}")

# Assign fixed x positions
x_positions = {
    "Basic": 1,
    "Ambiguous": 2,
    "Trajectory": 3
}

# Generate slight jitter for visibility
def jitter_x(center, count):
    step = 0.1
    start = center - step * (count - 1) / 2
    return [start + i * step for i in range(count)]

x_basic = jitter_x(x_positions["Basic"], len(basic_tokens))
x_ambiguous = jitter_x(x_positions["Ambiguous"], len(ambiguous_tokens))
x_trajectory = jitter_x(x_positions["Trajectory"], len(trajectory_tokens))

# Plot
plt.figure(figsize=(10, 6))
plt.scatter(x_basic, basic_tokens, label="Basic", s=80)
plt.scatter(x_ambiguous, ambiguous_tokens, label="Ambiguous", s=80)
plt.scatter(x_trajectory, trajectory_tokens, label="Trajectory", s=80)

plt.xticks([1, 2, 3], ["Basic", "Ambiguous", "Trajectory"])
plt.ylabel("Token Count")
plt.title("Token Count per Plan in Each Group")
plt.grid(True, linestyle='--', alpha=0.5)
plt.legend()
plt.tight_layout()
# save the plot
save_dir = Path("/home/jingyang/robosuite/myCode/my_planning_app/logs/")
# output_path = save_dir / "symbolic_plan_token_counts.png"
# plt.savefig(output_path, dpi=300, bbox_inches='tight')
plt.show()
