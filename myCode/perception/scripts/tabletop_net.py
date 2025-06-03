import torch
import torch.nn as nn

class TabletopNet(nn.Module):
    def __init__(self, num_objects=5, backbone_out=128):
        super(TabletopNet, self).__init__()
        self.num_objects = num_objects
        
        # Backbone CNN
        self.backbone = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, stride=2, padding=1),  # 112x112
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),  # 56x56
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),  # 28x28
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),  # Global pooling
        )
        
        # Shared feature vector
        self.fc_shared = nn.Linear(128, backbone_out)
        
        # Heads (per object)
        # self.fc_presence = nn.Linear(backbone_out, num_objects)
        self.fc_shape = nn.Linear(backbone_out, num_objects * 2)  # 2 classes: cube, cylinder
        self.fc_color = nn.Linear(backbone_out, num_objects * 3)  # 3 classes: red, green, blue
        self.fc_position = nn.Linear(backbone_out, num_objects * 3)  # x, y, z
        
    def forward(self, x):
        batch_size = x.size(0)
        feat = self.backbone(x).view(batch_size, -1)
        feat = self.fc_shared(feat)
        
        # presence = torch.sigmoid(self.fc_presence(feat))  # [B, num_objects]
        shape_logits = self.fc_shape(feat).view(batch_size, self.num_objects, 2)  # [B, num_objects, 2]
        color_logits = self.fc_color(feat).view(batch_size, self.num_objects, 3)  # [B, num_objects, 3]
        position = self.fc_position(feat).view(batch_size, self.num_objects, 3)  # [B, num_objects, 3]
        
        return shape_logits, color_logits, position

# Example forward pass
net = TabletopNet(num_objects=5)
dummy_input = torch.randn(4, 3, 224, 224)  # Batch of 4 images

shape_logits, color_logits, position = net(dummy_input)

# print("Presence:", presence.shape)         # [4, 5]
print("Shape logits:", shape_logits.shape) # [4, 5, 2]
print("Color logits:", color_logits.shape) # [4, 5, 3]
print("Position:", position.shape)         # [4, 5, 3]
