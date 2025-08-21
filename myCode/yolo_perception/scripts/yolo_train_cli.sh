#!/bin/bash
# yolo_train_cli.sh

echo "Starting YOLO training..."

# Mini-training: 160 images for mini-training, 20 images for validation
echo "Running mini training..."
yolo task=detect mode=train model=yolo11n.pt data=mini_tabletop.yaml epochs=100 imgsz=512 name=mini_tabletop_train device=0 verbose=True

# Full training: 4000 images for training, 500 for validation, 500 for testing
echo "Running full training..."
yolo task=detect mode=train model=yolo11n.pt data=full_tabletop.yaml epochs=100 imgsz=512 name=full_tabletop_train device=0 verbose=True

echo "Training completed!"