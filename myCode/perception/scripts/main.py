import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split
from perception_dataset import PerceptionDataset
from tabletop_net import TabletopNet
from tqdm import tqdm

def train_one_epoch(model, dataloader, criterion_shape, criterion_color, criterion_pos, optimizer, device):
    model.train()
    total_loss = 0.0
    progress_bar = tqdm(dataloader, desc="Train", unit='batches', leave=False)
    for images, shapes, colors, positions in progress_bar:
        images = images.to(device)
        shapes = shapes.to(device)
        colors = colors.to(device)
        positions = positions.to(device)

        optimizer.zero_grad()
        shape_logits, color_logits, pred_positions = model(images)

        # Compute losses
        loss_shape = criterion_shape(shape_logits.view(-1, 2), shapes.view(-1))
        loss_color = criterion_color(color_logits.view(-1, 3), colors.view(-1))
        loss_pos = criterion_pos(pred_positions, positions)

        loss = loss_shape + loss_color + loss_pos
        loss.backward()
        optimizer.step()

        total_loss += loss.item()

    avg_loss = total_loss / len(dataloader)
    return avg_loss

def validate_one_epoch(model, dataloader, criterion_shape, criterion_color, criterion_pos, device):
    model.eval()
    total_loss = 0.0

    with torch.no_grad():
        progress_bar = tqdm(dataloader, desc="Val", unit='batches', leave=False)
        for images, shapes, colors, positions in progress_bar:
            images = images.to(device)
            shapes = shapes.to(device)
            colors = colors.to(device)
            positions = positions.to(device)

            shape_logits, color_logits, pred_positions = model(images)

            # Compute losses
            loss_shape = criterion_shape(shape_logits.view(-1, 2), shapes.view(-1))
            loss_color = criterion_color(color_logits.view(-1, 3), colors.view(-1))
            loss_pos = criterion_pos(pred_positions, positions)

            loss = loss_shape + loss_color + loss_pos
            total_loss += loss.item()

    avg_loss = total_loss / len(dataloader)
    return avg_loss

def main():
    # Config
    home_dir = os.path.expanduser('~')
    data_dir = os.path.join(home_dir, 'robosuite', 'myCode', 'perception', 'data')
    num_objects = 5
    batch_size = 32
    num_epochs = 20
    learning_rate = 1e-4
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

    # Losses
    criterion_shape = nn.CrossEntropyLoss()
    criterion_color = nn.CrossEntropyLoss()
    criterion_pos = nn.MSELoss()

    # Optimizer
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)

    best_val_loss = float('inf')

    # Training loop
    # Training loop with tqdm
    for epoch in range(num_epochs):
        train_loss = train_one_epoch(
            model, train_loader, criterion_shape, criterion_color, criterion_pos, optimizer, device
        )
        val_loss = validate_one_epoch(
            model, val_loader, criterion_shape, criterion_color, criterion_pos, device
        )

        tqdm.write(f"Epoch [{epoch + 1}/{num_epochs}] → Train Loss: {train_loss:.4f}, Val Loss: {val_loss:.4f}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            os.makedirs('checkpoints', exist_ok=True)
            torch.save(model.state_dict(), 'checkpoints/best_model.pth')
            tqdm.write("Best model updated and saved.")

    torch.save(model.state_dict(), 'checkpoints/final_model.pth')
    print("Training completed and final model saved.")

if __name__ == "__main__":
    main()
