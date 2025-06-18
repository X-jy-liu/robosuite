from ultralytics import YOLO
from pathlib import Path

# Paths
model_path = "/home/s2644572/robosuite/myCode/yolo_perception/runs/detect/train/weights/best.pt"
test_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/images")
output_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/pred_images")
txt_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/pred_labels")
# Create output directories
output_dir.mkdir(parents=True, exist_ok=True)
txt_dir.mkdir(parents=True, exist_ok=True)

# Load model
model = YOLO(model_path)

num_images = 10 # adjust as needed
image_paths = sorted([p for p in test_dir.glob("*.jpg")] + [p for p in test_dir.glob("*.png")])[:num_images]

# Inference + save
for img_path in image_paths:
    result = model(img_path)[0]
    result.save(filename=output_dir / img_path.name)  # Save image with drawn boxes

    # Extract box data
    boxes = result.boxes.xywh.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy()

    lines = []
    for xywh, cls in zip(boxes, classes):
        x, y, w, h = xywh
        class_id = int(cls)
        line = f"{class_id} {x:.6f} {y:.6f} {w:.6f} {h:.6f}"
        lines.append(line)

    # Save as .txt in YOLO format
    txt_path = txt_dir / f"{img_path.stem}.txt"
    with open(txt_path, "w") as f:
        f.write("\n".join(lines))

print(f"Saved predictions and YOLO-format .txt files for {len(image_paths)} images:")
print(f" - Images: {output_dir}")
print(f" - Labels: {txt_dir}")