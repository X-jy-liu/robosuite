import os
import shutil

# Set paths
src_img_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/images/train'
src_lbl_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/labels/train'
dst_img_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/mini_tabletop/images/train'
dst_lbl_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/mini_tabletop/labels/train'
# src_img_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/images/val'
# src_lbl_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/labels/val'
# dst_img_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/mini_tabletop/images/val'
# dst_lbl_dir = '/home/s2644572/robosuite/myCode/yolo_perception/data/mini_tabletop/labels/val'

# total images: 5000, split into train, val and test in ratio of 80:10:10
# train: 4000, val: 500, test: 500

# Create destination folders if they don't exist
number_of_images = 4000
os.makedirs(dst_img_dir, exist_ok=True)
os.makedirs(dst_lbl_dir, exist_ok=True)
image_files = sorted(f for f in os.listdir(src_img_dir) if f.endswith('.png'))[:number_of_images]

# Copy images and corresponding labels
for img_file in image_files:
    lbl_file = img_file.replace('.png', '.txt')
    shutil.copy(os.path.join(src_img_dir, img_file), os.path.join(dst_img_dir, img_file))
    shutil.copy(os.path.join(src_lbl_dir, lbl_file), os.path.join(dst_lbl_dir, lbl_file))

print(f"✅ Copied {number_of_images} images and labels.")
