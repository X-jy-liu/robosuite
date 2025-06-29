import matplotlib.pyplot as plt

# Reference points
reference_points = {
        "point_1": [
            -0.1, 
            -0.1
        ],
        "point_2": [
            0.1, 
            -0.1
        ],
        "point_3": [
            0.0, 
            -0.2
        ],
        "point_4": [
            0.05, 
            0.2
        ],
        "point_5": [
            -0.29, 
            0.0
        ]
}
# Environment objects
objects = [
    {"name": "obj0", "shape": "cube", "color": "red", "position": [-0.2, -0.2], "size": 0.05},
    {"name": "obj1", "shape": "cube", "color": "blue", "position": [0.2, -0.2], "size": 0.05},
    {"name": "obj2", "shape": "cube", "color": "green", "position": [-0.2, 0.2], "size": 0.05},
    {"name": "obj3", "shape": "cylinder", "color": "red", "position": [0.15, 0.15], "size": 0.05},
    {"name": "obj4", "shape": "cylinder", "color": "blue", "position": [0.0, 0.0], "size": 0.05}
]
# objects = [
#       { "name": "obj0", "shape": "cube", "color": "green", "position": [-0.101, 0.058], "size": 0.05 },
#       { "name": "obj1", "shape": "cylinder", "color": "green", "position": [-0.22, 0.173], "size": 0.05 },
#       { "name": "obj2", "shape": "cube", "color": "green", "position": [0.018, 0.163], "size": 0.05 },
#       { "name": "obj3", "shape": "cube", "color": "green", "position": [0.186, 0.146], "size": 0.05 },
#       { "name": "obj4", "shape": "cube", "color": "green", "position": [-0.093, 0.191], "size": 0.05 }
#     ]

fig, ax = plt.subplots(figsize=(6, 6))

# Plot reference points
for label, coord in reference_points.items():
    ax.plot(coord[0], coord[1], 'kx')  # black 'x' for reference points
    ax.text(coord[0] + 0.01, coord[1] + 0.01, label, fontsize=9, color='black')

# Plot objects
for obj in objects:
    y, x = obj["position"]
    size = obj["size"]
    if obj["shape"] == "cube":
        square = plt.Rectangle((x - size/2, y - size/2), size, size, color=obj["color"], label=obj["name"])
        ax.add_patch(square)
    elif obj["shape"] == "cylinder":
        circle = plt.Circle((x, y), size / 2, color=obj["color"], label=obj["name"])
        ax.add_patch(circle)
    ax.text(x + 0.01, y + 0.01, obj["name"], fontsize=9, color='black')

ax.set_xlim(-0.4, 0.4)
ax.set_ylim(0.4, -0.4)
ax.set_aspect('equal')
ax.set_title("Reference Points and Environment Objects")
plt.grid(True)
plt.show()
