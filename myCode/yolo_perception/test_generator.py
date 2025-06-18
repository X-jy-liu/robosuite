import os
import shutil
from pathlib import Path
from tqdm import tqdm

def collect_dataset(input_dir, output_dir):
    input_dir = Path(input_dir)
    output_dir = Path(output_dir)
    image_extensions = ['.jpg', '.png', '.jpeg']

    # Create output directories
    (output_dir / "images").mkdir(parents=True, exist_ok=True)
    (output_dir / "labels_json").mkdir(parents=True, exist_ok=True)

    # Collect and copy matched pairs
    image_files = [f for f in input_dir.iterdir() 
                   if f.suffix.lower() in image_extensions and 
                   (input_dir / f.with_suffix('.json').name).exists()]

    print(f"Found {len(image_files)} matched image-label (.json) pairs.")

    for img_path in tqdm(image_files, desc="Copying files"):
        json_path = input_dir / img_path.with_suffix('.json').name
        shutil.copy2(img_path, output_dir / "images" / img_path.name)
        shutil.copy2(json_path, output_dir / "labels_json" / json_path.name)

    print(f"Copied {len(image_files)} pairs to '{output_dir}'.")

if __name__ == "__main__":
    collect_dataset(
        input_dir="/home/s2644572/robosuite/myCode/perception/data/test",   # Source with .jpg/.json files
        output_dir="/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test"  # Single destination folder
    )
