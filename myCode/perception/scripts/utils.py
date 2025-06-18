import torch.nn as nn
import matplotlib.pyplot as plt
import os

class PerceptionLoss(nn.Module):
    def __init__(self, weight_shape=1.0, weight_color=1.0, weight_position=10.0):
        super().__init__()
        self.shape_loss_fn = nn.CrossEntropyLoss()
        self.color_loss_fn = nn.CrossEntropyLoss()
        self.position_loss_fn = nn.MSELoss()

        self.weight_shape = weight_shape
        self.weight_color = weight_color
        self.weight_position = weight_position

    def forward(self, preds, targets):
        """
        Args:
            preds: tuple of (shape_logits, color_logits, positions)
                - shape_logits: (B, N, 2)
                - color_logits: (B, N, 3)
                - positions:    (B, N, 3)

            targets: tuple of (shape_labels, color_labels, pos_labels)
                - shape_labels: (B, N)
                - color_labels: (B, N)
                - pos_labels:   (B, N, 3)
        """
        shape_logits, color_logits, positions = preds
        shape_labels, color_labels, pos_labels = targets

        B, N, _ = shape_logits.shape

        # Flatten for CrossEntropy: input (B*N, C), target (B*N)
        loss_shape = self.shape_loss_fn(shape_logits.view(B*N, -1), shape_labels.view(B*N))
        loss_color = self.color_loss_fn(color_logits.view(B*N, -1), color_labels.view(B*N))
        loss_pos = self.position_loss_fn(positions, pos_labels)

        total = (self.weight_shape * loss_shape +
                 self.weight_color * loss_color +
                 self.weight_position * loss_pos)

        return total, {
            'loss_shape': loss_shape.item(),
            'loss_color': loss_color.item(),
            'loss_pos': loss_pos.item()
        }

def error_plot(train_errors, val_errors, num_epochs, plot_name, if_save=True):

    plt.figure(figsize=(10, 5))
    plt.plot(range(1, num_epochs + 1), train_errors, label='Train Loss', marker='o')
    plt.plot(range(1, num_epochs + 1), val_errors, label='Validation Loss', marker='x')
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.title('Training and Validation Loss')
    plt.legend()
    plt.grid()
    
    if if_save:
        plot_dir = '/home/s2644572/robosuite/myCode/perception/plots/'
        os.makedirs(plot_dir, exist_ok=True)
        save_path = os.path.join(plot_dir, plot_name + '.png')
        plt.savefig(save_path)
        print(f"Plot saved as {save_path}")
    
    plt.show()

# class MultiObjectLift(Lift):
#     def __init__(self, predefined_objects=None, **kwargs):
#         self._predefined_objects = predefined_objects  # store user input
#         super().__init__(**kwargs)
#     def _load_model(self):
#         super()._load_model()
#         table_z = self.table_offset[2]
#         self.robots[0].robot_model.set_base_xpos([-2, 0, 0])
#         half_height = 0.0125  # Half height for objects
#         cylinder_calibration = 0.0 # Calibration offset for cylinder height
#         # checking the height of the table
#         print(f"Table height: {table_z}")
#         self.object_metadata = []  # Store object info for later access
#         self.COLORS = {
#             "red": [1, 0, 0, 1],
#             "green": [0, 1, 0, 1],
#             "blue": [0, 0, 1, 1]
#         }
#         # Predefined objects: (name, shape, color_name, position)
#         # Use user-supplied predefined_objects or fallback to default
#         if self._predefined_objects is not None:
#             predefined_objects = self._predefined_objects
#         else:
#             predefined_objects = [
#                 ("obj0", "cube", "red", [-0.2, -0.2, table_z + half_height]),
#                 ("obj1", "cube", "blue", [0.2, -0.2, table_z + half_height]),
#                 ("obj2", "cube", "green", [-0.2, 0.2, table_z + half_height]),
#                 ("obj3", "cylinder", "red", [0.15, 0.15, table_z + 2*half_height + cylinder_calibration]),
#                 ("obj4", "cylinder", "blue", [0.05, -0.05, table_z + 2*half_height + cylinder_calibration]),
#             ]

#         for name, shape, color_name, pos in predefined_objects:
#             color_rgba = self.COLORS[color_name]

#             if shape == "cube":
#                 obj = BoxObject(
#                     name=name,
#                     size=[0.025, 0.025, 0.025],
#                     rgba=color_rgba,
#                     material=None,
#                     obj_type="all"
#                 )
#             else:
#                 obj = CylinderObject(
#                     name=name,
#                     size=[0.025, 0.025],  # (radius, height)
#                     rgba=color_rgba,
#                     material=None,
#                     obj_type="all"
#                 )

#             obj.get_obj().set("pos", f"{pos[0]} {pos[1]} {pos[2]}")

#             # Add to the model
#             self.model.merge_objects([obj])
#             add_to_dict(self.model.worldbody, "body", obj.get_obj())

#             # save the object metadata
#             self.object_metadata.append({
#                 "name": name,
#                 "shape": shape,
#                 "color": color_name,
#                 "position": pos
#             })

#     def _setup_camera(self):
#         cam_id = self.sim.model.camera_name2id("birdview")
#         self.sim.model.cam_pos[cam_id] = [0.0, 0.0, 1.5]  # Adjust height as needed
#         self.sim.model.cam_quat[cam_id] = [1.0, 0.0, 0.0, 0.0]  # Look straight down
#         self.sim.model.cam_fovy[cam_id] = 60  # Smaller FOV to zoom in tighter

# # function to render the scene from the predicted objects
# def render_scene(predicted_objects, controller):
#     env = MultiObjectLift(
#             robots="Panda",
#             predefined_objects=predicted_objects,
#             controller_configs=controller,
#             has_renderer=True,
#             camera_names="birdview",
#             camera_heights=512,
#             camera_widths=512,
#             camera_depths=False
#         )
#     obs = env.reset()
