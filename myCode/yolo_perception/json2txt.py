import json
import os
from tqdm import tqdm

# Configuration
BOX_WIDTH = 0.05 / 0.8  # Normalized width for YOLO
BOX_HEIGHT = 0.05 / 0.8  # Normalized height for YOLO

classes = {
    "red_cube": 0,
    "green_cube": 1,
    "blue_cube": 2,
    "red_cylinder": 3,
    "green_cylinder": 4,
    "blue_cylinder": 5,
}

def convert_label(json_path, output_txt_path):
    with open(json_path, 'r') as f:
        objects = json.load(f)

    lines = []
    for obj in objects:
        shape = obj["shape"]
        color = obj["color"]
        pos_x, pos_y, _ = obj["position"]

        key = f"{color}_{shape}"
        class_id = classes.get(key, -1)
        if class_id == -1:
            print(f"Unknown class: {key}")
            continue

        # Convert from [-0.4, 0.4] to [0, 1] because the table size is 0.8x0.8 and the center is at (0, 0)
        x_center = (pos_x + 0.4) / 0.8
        y_center = (pos_y + 0.4) / 0.8

        line = f"{class_id} {x_center:.6f} {y_center:.6f} {BOX_WIDTH:.6f} {BOX_HEIGHT:.6f}"
        lines.append(line)

    # Write to YOLO .txt file
    with open(output_txt_path, 'w') as f:
        f.write("\n".join(lines))

if __name__ == "__main__":
    json_dir = "/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/labels_json/train"
    output_dir = "/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/labels/train"

    # json_dir = "/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/labels_json/val"
    # output_dir = "/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/labels/val"

    # json_dir = "/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/labels_json"
    # output_dir = "/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/labels"

    os.makedirs(output_dir, exist_ok=True)

    for json_file in tqdm(os.listdir(json_dir),desc="Converting JSON to TXT", unit="files"):
        if json_file.endswith('.json'):
            json_path = os.path.join(json_dir, json_file)
            output_txt_path = os.path.join(output_dir, json_file.replace('.json', '.txt'))
            convert_label(json_path, output_txt_path)
            # break
            # print(f"Converted {json_file} to {output_txt_path}")