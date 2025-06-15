import torch
import torch.nn as nn
import torchvision.models as models

class PerceptionNet(nn.Module):
    def __init__(self, num_objects=5, num_shape_classes=2, num_color_classes=3):
        super(PerceptionNet, self).__init__()
        self.num_objects = num_objects
        self.num_shape_classes = num_shape_classes
        self.num_color_classes = num_color_classes
        
        # Backbone: ResNet18 without final classification layer
        resnet = models.resnet18(pretrained=True)
        self.backbone = nn.Sequential(*list(resnet.children())[:-1])  # Output: (B, 512, 1, 1) remove the last fc layer
        self.feature_dim = 512

        # Heads
        self.shape_head = nn.Linear(self.feature_dim, num_objects * num_shape_classes)
        self.color_head = nn.Linear(self.feature_dim, num_objects * num_color_classes)
        # self.pos_head   = nn.Linear(self.feature_dim, num_objects * 3)  # x, y, z
        # self.shape_head = nn.Sequential(
        #     nn.Linear(512, 256),
        #     nn.ReLU(),
        #     nn.Linear(256, num_objects * num_shape_classes)
        # )
        # self.color_head = nn.Sequential(
        #     nn.Linear(512, 256),
        #     nn.ReLU(),
        #     nn.Linear(256, num_objects * num_color_classes)
        # )

        self.pos_head = nn.Sequential(
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Linear(256, num_objects * 3)
        )
        
    def forward(self, x):
        # Backbone features
        features = self.backbone(x)               # (B, 512, 1, 1)
        features = features.view(features.size(0), -1)  # (B, 512)

        # Predict shape logits
        shape_logits = self.shape_head(features)         # (B, N * C_shape)
        shape_logits = shape_logits.view(-1, self.num_objects, self.num_shape_classes)

        # Predict color logits
        color_logits = self.color_head(features)         # (B, N * C_color)
        color_logits = color_logits.view(-1, self.num_objects, self.num_color_classes)

        # Predict positions
        positions = self.pos_head(features)              # (B, N * 3)
        positions = positions.view(-1, self.num_objects, 3)

        return shape_logits, color_logits, positions

# Example usage
if __name__ == '__main__':
    model = PerceptionNet(num_objects=5, num_shape_classes=2, num_color_classes=3)
    print(model)

    # Dummy input tensor (batch size 1, 3 channels, 224x224 image)
    input_tensor = torch.randn(1, 3, 224, 224)
    
    # Forward pass
    shape_logits, color_logits, positions = model(input_tensor)
    
    print("Shape logits:", shape_logits.shape)   # Should be [1, 5, 2]
    print("Color logits:", color_logits.shape)   # Should be [1, 5, 3]
    print("Positions:", positions.shape)         # Should be [1, 5, 3]
