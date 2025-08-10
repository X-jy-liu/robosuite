from ultralytics import YOLO
import cv2
from pathlib import Path

# Paths
model_path = "/home/s2644572/robosuite/myCode/yolo_perception/runs/detect/mini_tabletop_train_1/weights/best.pt"
test_img_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/realistic_noise")
# ref_txt_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/labels")
# Output directories for predictions
output_img_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/inference_from_mini_train_realistic_noise/images")
pred_txt_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/inference_from_mini_train_realistic_noise/labels")
# Create output directories
output_img_dir.mkdir(parents=True, exist_ok=True)
pred_txt_dir.mkdir(parents=True, exist_ok=True)

# Load model
model = YOLO(model_path)

num_images = 500 # adjust as needed
image_paths = sorted([p for p in test_img_dir.glob("*.jpg")] + [p for p in test_img_dir.glob("*.png")])[:num_images]

# initialize parameters for converting yolo x y pixels to meters
img_size = 512
table_size_m = 0.8  # size of the tabletop in meters
meters_per_pixel = table_size_m / img_size
fixed_box_size_m = 0.05

for img_path in image_paths:
    result = model(img_path)[0]
    # Save rendered image with boxes
    rendered = result.plot()  # returns numpy array with boxes drawn
    output_img_path = output_img_dir / img_path.name
    cv2.imwrite(str(output_img_path), rendered)

    # Extract prediction data
    boxes = result.boxes.xywh.cpu().numpy()  # x_center, y_center, width, height (in pixels)
    classes = result.boxes.cls.cpu().numpy()

    xywh_norm_lines = []
    for xywh, cls in zip(boxes, classes):
        x_pixel, y_pixel, _, _ = xywh
        class_id = int(cls)

        # === normalize pixel coordinates ===
        x_norm = x_pixel / img_size
        y_norm = y_pixel / img_size
        norm_w = fixed_box_size_m / table_size_m
        norm_h = fixed_box_size_m / table_size_m
        xywh_norm_lines.append(f"{class_id} {x_norm:.6f} {y_norm:.6f} {norm_w:.6f} {norm_h:.6f}")


    # Save calibrated labels (in meters)
    txt_path = pred_txt_dir / f"{img_path.stem}.txt"
    with open(txt_path, "w") as f:
        f.write("\n".join(xywh_norm_lines))

print(f"✅ Saved calibrated predictions for {len(image_paths)} images.")
print(f" - Calibrated Images: {output_img_dir}")
print(f" - Labels in meters: {pred_txt_dir}")