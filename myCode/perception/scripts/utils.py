import torch
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