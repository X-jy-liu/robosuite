import cv2
import matplotlib.pyplot as plt
# === Set your paths ===
image_path = "/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/images/train/scene_1.png"
label_path = "/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/labels/train/scene_1.txt"  # same name as image but .txt
image_size = 512  # width and height of the image

# (Optional) Map class IDs to names
class_names = {
    0: 'red_cube',
    1: 'green_cube',
    2: 'blue_cube',
    3: 'red_cylinder',
    4: 'green_cylinder',
    5: 'blue_cylinder'
}

# === Load image ===
img = cv2.imread(image_path)
if img is None:
    raise FileNotFoundError(f"Image not found: {image_path}")

# === Read and draw labels ===
with open(label_path, "r") as f:
    for line in f:
        class_id, x_center, y_center, w, h = map(float, line.strip().split())
        class_id = int(class_id)

        # Convert from normalized to pixel coords
        x_center *= image_size
        y_center *= image_size
        w *= image_size
        h *= image_size

        x1 = int(x_center - w / 2)
        y1 = int(y_center - h / 2)
        x2 = int(x_center + w / 2)
        y2 = int(y_center + h / 2)

        # Draw box
        color = (0, 255, 0)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        # Draw label
        label = class_names.get(class_id, str(class_id))
        cv2.putText(img, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

# === Show image ===
# Convert BGR (OpenCV) to RGB (matplotlib)
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# Show image with matplotlib
plt.figure(figsize=(6, 6))
plt.imshow(img_rgb)
plt.title("Labeled Image")
plt.axis("off")
plt.show()
