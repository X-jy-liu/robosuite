import os
import shutil

# Set paths
# src_img_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/images/train'
# src_lbl_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/labels/train'
# dst_img_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/mini_tabletop/images/train'
# dst_lbl_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/mini_tabletop/labels/train'
src_img_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/images/val'
src_lbl_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/labels/val'
dst_img_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/mini_tabletop/images/val'
dst_lbl_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/mini_tabletop/labels/val'

# Create destination folders if they don't exist
os.makedirs(dst_img_dir, exist_ok=True)
os.makedirs(dst_lbl_dir, exist_ok=True)

# Get the first 20 image filenames (sorted for consistency)
image_files = sorted(f for f in os.listdir(src_img_dir) if f.endswith('.png'))[:20]

# Copy images and corresponding labels
for img_file in image_files:
    lbl_file = img_file.replace('.png', '.txt')
    shutil.copy(os.path.join(src_img_dir, img_file), os.path.join(dst_img_dir, img_file))
    shutil.copy(os.path.join(src_lbl_dir, lbl_file), os.path.join(dst_lbl_dir, lbl_file))

print("✅ Copied 20 images and labels.")
