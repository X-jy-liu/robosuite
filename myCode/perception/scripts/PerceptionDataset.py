import os
import json
import torch
from torch.utils.data import Dataset
from PIL import Image
import torchvision.transforms as T

class PerceptionDataset(Dataset):
    def __init__(self, data_dir, num_objects=5, transform=None):
        self.data_dir = data_dir
        self.image_files = sorted([f for f in os.listdir(data_dir) if f.endswith('.png')])
        self.num_objects = num_objects
        self.transform = transform or T.Compose([
            T.Resize((224, 224)),
            T.ToTensor()
        ])
        # Map strings to indices
        self.shape_map = {'cube': 0, 'cylinder': 1}
        self.color_map = {'red': 0, 'green': 1, 'blue': 2}

    def __len__(self):
        return len(self.image_files)

    def __getitem__(self, idx):
        image_filename = self.image_files[idx]
        image_path = os.path.join(self.data_dir, image_filename)
        json_path = image_path.replace('.png', '.json')

        # Load image
        image = Image.open(image_path).convert('RGB')
        image = self.transform(image)

        # Load labels
        with open(json_path, 'r') as f:
            labels = json.load(f)

        # Initialize tensors
        # presence = torch.zeros(self.num_objects)
        shape = torch.zeros(self.num_objects, dtype=torch.long)
        color = torch.zeros(self.num_objects, dtype=torch.long)
        position = torch.zeros(self.num_objects, 3)

        for i, obj in enumerate(labels):
            if i >= self.num_objects:
                break  # Ignore extra objects
            # presence[i] = 1  # Mark object present
            shape[i] = self.shape_map[obj['shape']]
            color[i] = self.color_map[obj['color']]
            position[i] = torch.tensor(obj['position'])

        return image, shape, color, position
    

# Example usage:
if __name__ == "__main__":
    dataset = PerceptionDataset(data_dir='/home/jingyang/robosuite/myCode/perception/data', num_objects=5)
    for i in range(len(dataset)):
        image, shape, color, position = dataset[i]
        print(f"Image {i}: Shape: {shape}, Color: {color}, Position: {position}")
        # Limit to 10 iterations for demonstration
        if i == 10:
            break

