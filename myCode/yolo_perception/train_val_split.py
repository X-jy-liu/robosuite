import os
import shutil
import random
from pathlib import Path

def split_dataset(input_dir, output_dir, val_ratio=0.2, seed=42):
    random.seed(seed)

    input_dir = Path(input_dir)
    output_dir = Path(output_dir)
    image_extensions = ['.jpg', '.png', '.jpeg']

    # Collect image files with matching .json labels
    image_files = [f for f in input_dir.iterdir() if f.suffix.lower() in image_extensions and (input_dir / f.with_suffix('.json').name).exists()]
    print(f"Found {len(image_files)} matched image-label (.json) pairs.")

    # Shuffle and split
    random.shuffle(image_files)
    val_count = int(len(image_files) * val_ratio)
    val_files = image_files[:val_count]
    train_files = image_files[val_count:]

    for split, files in [("train", train_files), ("val", val_files)]:
        (output_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (output_dir / "labels_json" / split).mkdir(parents=True, exist_ok=True)

        for img_path in files:
            json_path = input_dir / img_path.with_suffix('.json').name
            shutil.copy2(img_path, output_dir / "images" / split / img_path.name)
            shutil.copy2(json_path, output_dir / "labels_json" / split / json_path.name)

    print(f"Split complete. Train: {len(train_files)}, Val: {len(val_files)}")

if __name__ == "__main__":
    split_dataset(
        input_dir="/home/s2644572/robosuite/myCode/perception/data/train_val",     # Folder with .jpg/.json files together
        output_dir="/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop",        # Output split folder
        val_ratio=0.2                      # 20% validation split
    )
