# cli training script

# yolo detect train model=yolov8n.yaml data=tabletop.yaml epochs=200 imgsz=512 optimizer=Adam name=from_scratch_adam device=0 verbose=True

# 4000 images for training, 500 for validation, 500 for testing
# yolo task=detect mode=train model=yolo11n.pt data=mini_tabletop.yaml epochs=100 imgsz=512 name=mini_tabletop_train_1 device=0 verbose=True
