import os
import shutil
import random

# Parameters
source_folder = '/home/s2644572/robosuite/myCode/perception/data'
output_folder = '/home/s2644572/robosuite/myCode/perception/data'
test_ratio = 0.2  # 2:8 split means 20% test

# Output folders
test_folder = os.path.join(output_folder, 'test')
train_val_folder = os.path.join(output_folder, 'train_val')
os.makedirs(test_folder, exist_ok=True)
os.makedirs(train_val_folder, exist_ok=True)

# Get base names (without extension) that have both .png and .json
all_files = os.listdir(source_folder)
base_names = set(os.path.splitext(f)[0].replace(" ", "_") for f in all_files)
pairs = []
for name in base_names:
    png = name + '.png'
    json = name + '.json'
    if png in all_files and json in all_files:
        pairs.append((png, json))

# Shuffle
random.shuffle(pairs)

# Split
total = len(pairs)
test_count = int(total * test_ratio)
test_pairs = pairs[:test_count]
train_val_pairs = pairs[test_count:]

# Function to move paired files
def move_pairs(pairs, destination):
    for img, jsn in pairs:
        shutil.move(os.path.join(source_folder, img), os.path.join(destination, img))
        shutil.move(os.path.join(source_folder, jsn), os.path.join(destination, jsn))

# Move files
move_pairs(test_pairs, test_folder)
move_pairs(train_val_pairs, train_val_folder)

print(f"Moved {len(test_pairs)} pairs to test.")
print(f"Moved {len(train_val_pairs)} pairs to train_val.")