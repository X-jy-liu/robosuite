from ultralytics import YOLO
import cv2
from pathlib import Path

# Paths
model_path = "/home/s2644572/robosuite/myCode/yolo_perception/runs/detect/from_scratch_adam/weights/best.pt"
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

# initialize parameters for converting yolo x y pixels to meters
img_size = 512
meters_per_pixel = 0.8 / img_size
fixed_box_size_m = 0.05

for img_path in image_paths:
    result = model(img_path)[0]

    # Load image
    img = cv2.imread(str(img_path))
    if img is None:
        print(f"Could not load image: {img_path}")
        continue

    # Extract prediction data
    boxes = result.boxes.xywh.cpu().numpy()  # x_center, y_center, width, height (in pixels)
    classes = result.boxes.cls.cpu().numpy()

    lines = []
    for xywh, cls in zip(boxes, classes):
        x_pixel, y_pixel, _, _ = xywh
        class_id = int(cls)

        # === Convert to meters ===
        x_meter = x_pixel * meters_per_pixel - 0.4
        y_meter = y_pixel * meters_per_pixel - 0.4
        w_meter = fixed_box_size_m
        h_meter = fixed_box_size_m

        lines.append(f"{class_id} {x_meter:.6f} {y_meter:.6f} {w_meter:.6f} {h_meter:.6f}")

        # === Visualize calibrated box ===
        # Convert meter coordinates back to pixel for visualization
        x_vis = int((x_meter + 0.4) / 0.8 * img_size)
        y_vis = int((y_meter + 0.4) / 0.8 * img_size)
        box_pixel_size = int(fixed_box_size_m / 0.8 * img_size)

        x1 = x_vis - box_pixel_size // 2
        y1 = y_vis - box_pixel_size // 2
        x2 = x_vis + box_pixel_size // 2
        y2 = y_vis + box_pixel_size // 2

        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img, f"cls {class_id}", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # Save calibrated labels (in meters)
    txt_path = txt_dir / f"{img_path.stem}.txt"
    with open(txt_path, "w") as f:
        f.write("\n".join(lines))

    # Save image with calibrated boxes
    cv2.imwrite(str(output_dir / img_path.name), img)

print(f"✅ Saved calibrated predictions for {len(image_paths)} images.")
print(f" - Calibrated Images: {output_dir}")
print(f" - Labels in meters: {txt_dir}")