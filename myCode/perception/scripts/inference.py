import torch
import json
import torch.nn.functional as F
from torchvision import transforms
from PIL import Image
from tabletop_net import TabletopNet  # assuming your model is saved as tabletop_net.py

# === 1. Load the model and checkpoint ===
def load_model(checkpoint_path, device='cpu', num_objects=5):
    model = TabletopNet(num_objects=num_objects)
    model.load_state_dict(torch.load(checkpoint_path, map_location=device))
    model.to(device)
    model.eval()
    return model

# === 2. Preprocess the input image ===
def preprocess_image(image_path):
    transform = transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),  # Converts to [0,1]
        transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])  # Optional: match training norm
    ])
    image = Image.open(image_path).convert('RGB')
    return transform(image).unsqueeze(0)  # Add batch dim: [1, 3, 224, 224]

# === 3. Run inference ===
def predict(model, input_tensor, device='cpu'):
    input_tensor = input_tensor.to(device)
    with torch.no_grad():
        shape_logits, color_logits, position = model(input_tensor)
        
        shape_probs = F.softmax(shape_logits, dim=-1)  # [B, num_objects, 2]
        color_probs = F.softmax(color_logits, dim=-1)  # [B, num_objects, 3]
        
        shape_preds = torch.argmax(shape_probs, dim=-1)  # [B, num_objects]
        color_preds = torch.argmax(color_probs, dim=-1)  # [B, num_objects]
        
    return shape_preds, color_preds, position

# === 4. Example usage ===
if __name__ == '__main__':
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    checkpoint_path = '/home/s2644572/robosuite/myCode/perception/checkpoints/final_model.pth'
    image_path = '/home/s2644572/robosuite/myCode/perception/data/test/scene_5.png'  # replace with your image

    model = load_model(checkpoint_path, device=device, num_objects=5)
    input_tensor = preprocess_image(image_path)
    shape, color, pos = predict(model, input_tensor, device)
    # Print the predictions
    print("Predicted Shapes:", shape)   # tensor([ [0, 1, 1, 0, 1] ]) e.g. 0=cube, 1=cylinder
    print("Predicted Colors:", color)   # tensor([ [2, 0, 1, 2, 0] ]) e.g. 0=red, 1=green, 2=blue
    print("Predicted Positions:", pos)  # tensor of shape [1, 5, 3]

    print('-'*20 + ' Reference Labels ' + '-'*20)
    # print the shape, color and position labels from the json file
    # Sample JSON input (replace with your file read)
    with open('myCode/perception/data/test/scene_5.json', 'r') as f:
        data = json.load(f)

    # Optional: pad to fixed object count (e.g. 5)
    max_objects = 5

    # Mappings
    shape2id = {'cube': 0, 'cylinder': 1}
    color2id = {'red': 0, 'green': 1, 'blue': 2}

    # Initialize empty lists
    positions = []
    shapes = []
    colors = []

    # Extract and encode
    for obj in data[:max_objects]:  # truncate if more than max_objects
        positions.append(obj['position'])
        shapes.append(shape2id[obj['shape']])
        colors.append(color2id[obj['color']])

    # Convert to tensors
    positions_tensor = torch.tensor([positions], dtype=torch.float32).to('cuda:0')  # shape [1, 5, 3]
    shapes_tensor = torch.tensor([shapes], dtype=torch.int64).to('cuda:0')         # shape [1, 5]
    colors_tensor = torch.tensor([colors], dtype=torch.int64).to('cuda:0')         # shape [1, 5]

    print("reference Shapes:", shapes_tensor)
    print("reference Colors:", colors_tensor)
    print("reference Positions:", positions_tensor)