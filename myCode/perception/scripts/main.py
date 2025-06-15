import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split
from perception_dataset import PerceptionDataset
from tabletop_net import TabletopNet
from tqdm import tqdm
from utils import error_plot, PerceptionLoss

def train_one_epoch(model, dataloader, criterion, optimizer, device):
    model.train()
    total_loss = 0.0
    progress_bar = tqdm(dataloader, desc="Train", unit='batches', leave=True)
    for images, shapes, colors, positions in progress_bar:
        images = images.to(device)
        shapes = shapes.to(device)
        colors = colors.to(device)
        positions = positions.to(device)

        optimizer.zero_grad()
        shape_logits, color_logits, pred_positions = model(images)

        # Unified loss computation
        loss, _ = criterion((shape_logits, color_logits, pred_positions),
                            (shapes, colors, positions))

        loss.backward()
        optimizer.step()

        total_loss += loss.item()

    avg_loss = total_loss / len(dataloader)
    return avg_loss

def validate_one_epoch(model, dataloader, criterion, device):
    model.eval()
    total_loss = 0.0

    with torch.no_grad():
        progress_bar = tqdm(dataloader, desc="Val", unit='batches', leave=True)
        for images, shapes, colors, positions in progress_bar:
            images = images.to(device)
            shapes = shapes.to(device)
            colors = colors.to(device)
            positions = positions.to(device)

            shape_logits, color_logits, pred_positions = model(images)

            # Unified loss computation
            loss, _ = criterion((shape_logits, color_logits, pred_positions),
                                (shapes, colors, positions))

            total_loss += loss.item()

    avg_loss = total_loss / len(dataloader)
    return avg_loss

def main():
    # Config
    home_dir = os.path.expanduser('~')
    data_dir = os.path.join(home_dir, 'robosuite', 'myCode', 'perception', 'data', 'train_val')
    num_objects = 5
    batch_size = 32
    num_epochs = 1
    learning_rate = 1e-4
    plot_name = 'test_plot' # without .png extension
    model_save_path = os.path.join(home_dir, 'robosuite', 'myCode', 'perception', 'checkpoints', 'test_model.pth')
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    # Dataset and DataLoader
    dataset = PerceptionDataset(data_dir, num_objects=num_objects)
    val_size = int(0.2 * len(dataset))
    train_size = len(dataset) - val_size
    train_dataset, val_dataset = random_split(dataset, [train_size, val_size])

    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size)

    # Model
    model = TabletopNet(num_objects=num_objects).to(device)

    # Loss function
    criterion = PerceptionLoss(weight_shape=1.0, weight_color=1.0, weight_position=10.0)

    # Optimizer
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)

    best_val_loss = float('inf')

    # initialize train and validation error lists
    train_errors = []
    val_errors = []

    # Training loop
    # Training loop with tqdm
    for epoch in range(num_epochs):
        tqdm.write(f"Epoch {epoch + 1}/{num_epochs} - Training...")
        train_loss = train_one_epoch(model, train_loader, criterion, optimizer, device)
        tqdm.write(f"Epoch {epoch + 1}/{num_epochs} - Validation...")
        val_loss = validate_one_epoch(model, val_loader, criterion, device)
        train_errors.append(train_loss)
        val_errors.append(val_loss)

        tqdm.write(f"Epoch [{epoch + 1}/{num_epochs}] → Train Loss: {train_loss:.4f}, Val Loss: {val_loss:.4f}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            os.makedirs('checkpoints', exist_ok=True)
            best_model = model.state_dict()
            tqdm.write("Best model updated.")

    # plot the training and validation errors
    error_plot(train_errors, val_errors, num_epochs, plot_name, if_save=True)
    torch.save(best_model, model_save_path)
    print("Training completed and final model saved.")

if __name__ == "__main__":
    main()
