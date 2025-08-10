from ultralytics import YOLO
import cv2
import numpy as np
from pathlib import Path

def calculate_iou(box1, box2):
    """
    Calculate Intersection over Union (IoU) between two boxes
    box format: [x_center, y_center, width, height]
    """
    # Convert to corner coordinates
    x1_min = box1[0] - box1[2] / 2
    y1_min = box1[1] - box1[3] / 2
    x1_max = box1[0] + box1[2] / 2
    y1_max = box1[1] + box1[3] / 2
    
    x2_min = box2[0] - box2[2] / 2
    y2_min = box2[1] - box2[3] / 2
    x2_max = box2[0] + box2[2] / 2
    y2_max = box2[1] + box2[3] / 2
    
    # Calculate intersection
    inter_x_min = max(x1_min, x2_min)
    inter_y_min = max(y1_min, y2_min)
    inter_x_max = min(x1_max, x2_max)
    inter_y_max = min(y1_max, y2_max)
    
    if inter_x_max <= inter_x_min or inter_y_max <= inter_y_min:
        return 0.0
    
    intersection = (inter_x_max - inter_x_min) * (inter_y_max - inter_y_min)
    
    # Calculate union
    area1 = box1[2] * box1[3]
    area2 = box2[2] * box2[3]
    union = area1 + area2 - intersection
    
    return intersection / union if union > 0 else 0.0

def apply_confidence_based_filtering(boxes, classes, confidences, iou_threshold=0.5):
    """
    Filter overlapping detections by keeping only the highest confidence box
    among boxes that have IoU above the threshold
    
    Args:
        boxes: numpy array of shape (N, 4) with [x_center, y_center, width, height]
        classes: numpy array of shape (N,) with class IDs
        confidences: numpy array of shape (N,) with confidence scores
        iou_threshold: IoU threshold for considering boxes as overlapping
    
    Returns:
        indices of boxes to keep
    """
    if len(boxes) == 0:
        return []
    
    # Create list of detections with indices
    detections = []
    for i in range(len(boxes)):
        detections.append({
            'index': i,
            'box': boxes[i],
            'class': classes[i],
            'confidence': confidences[i]
        })
    
    # Find overlapping groups
    overlapping_groups = []
    processed = set()
    
    for i, detection1 in enumerate(detections):
        if i in processed:
            continue
            
        # Start a new group with current detection
        current_group = [detection1]
        processed.add(i)
        
        # Find all detections that overlap with any detection in current group
        for j, detection2 in enumerate(detections):
            if j in processed or j == i:
                continue
                
            # Check if detection2 overlaps with any detection in current group
            overlaps_with_group = False
            for group_detection in current_group:
                # Only check overlap if same class
                if detection2['class'] == group_detection['class']:
                    iou = calculate_iou(detection2['box'], group_detection['box'])
                    if iou >= iou_threshold:
                        overlaps_with_group = True
                        break
            
            if overlaps_with_group:
                current_group.append(detection2)
                processed.add(j)
        
        overlapping_groups.append(current_group)
    
    # For each group, keep only the detection with highest confidence
    keep_indices = []
    for group in overlapping_groups:
        if len(group) == 1:
            # Single detection, always keep
            keep_indices.append(group[0]['index'])
        else:
            # Multiple overlapping detections, keep the one with highest confidence
            best_detection = max(group, key=lambda x: x['confidence'])
            keep_indices.append(best_detection['index'])
            
            # Print info about the filtering for debugging
            confidences_in_group = [det['confidence'] for det in group]
            print(f"    Overlapping group of {len(group)} boxes (confidences: {[f'{c:.3f}' for c in confidences_in_group]}) -> kept best: {best_detection['confidence']:.3f}")
    
    return sorted(keep_indices)

# Paths
model_path = "/home/s2644572/robosuite/myCode/yolo_perception/runs/detect/mini_tabletop_train_1/weights/best.pt"
test_img_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/noised_images")

# Output directories for predictions
output_img_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/inference_from_mini_train_1_noised_nms/images")
pred_txt_dir = Path("/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/inference_from_mini_train_1_noised_nms/labels")

# Create output directories
output_img_dir.mkdir(parents=True, exist_ok=True)
pred_txt_dir.mkdir(parents=True, exist_ok=True)

# Load model
model = YOLO(model_path)

# NMS parameters
iou_threshold = 0.7  # IoU threshold for considering boxes as overlapping (higher = less aggressive grouping)
confidence_threshold = 0.1  # Minimum confidence to consider a detection (lower since we're using confidence for final selection)

num_images = 500  # adjust as needed
image_paths = sorted([p for p in test_img_dir.glob("*.jpg")] + [p for p in test_img_dir.glob("*.png")])[:num_images]

# Initialize parameters for converting yolo x y pixels to meters
img_size = 512
table_size_m = 0.8  # size of the tabletop in meters
meters_per_pixel = table_size_m / img_size
fixed_box_size_m = 0.05

total_before_nms = 0
total_after_nms = 0

for img_path in image_paths:
    result = model(img_path)[0]
    
    # Extract prediction data
    if result.boxes is not None and len(result.boxes) > 0:
        boxes = result.boxes.xywh.cpu().numpy()  # x_center, y_center, width, height (in pixels)
        classes = result.boxes.cls.cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()  # confidence scores
        
        print(f"Image: {img_path.name}")
        print(f"  Before NMS: {len(boxes)} detections")
        total_before_nms += len(boxes)
        
        # Filter by confidence threshold first
        confidence_mask = confidences >= confidence_threshold
        filtered_boxes = boxes[confidence_mask]
        filtered_classes = classes[confidence_mask]
        filtered_confidences = confidences[confidence_mask]
        
        print(f"  After confidence filtering (>{confidence_threshold}): {len(filtered_boxes)} detections")
        
        if len(filtered_boxes) > 0:
            # Apply confidence-based filtering for overlapping boxes
            keep_indices = apply_confidence_based_filtering(filtered_boxes, filtered_classes, filtered_confidences, iou_threshold)
            
            # Keep only the selected detections
            final_boxes = filtered_boxes[keep_indices]
            final_classes = filtered_classes[keep_indices]
            final_confidences = filtered_confidences[keep_indices]
            
            print(f"  After confidence-based filtering (IoU>{iou_threshold}): {len(final_boxes)} detections")
            total_after_nms += len(final_boxes)
            
            # Create new result object for visualization (optional)
            # Note: This is a simplified approach for saving the rendered image
            # For exact visualization, you might need to modify the result object
            
            xywh_norm_lines = []
            for i, (xywh, cls, conf) in enumerate(zip(final_boxes, final_classes, final_confidences)):
                x_pixel, y_pixel, _, _ = xywh
                class_id = int(cls)

                # === normalize pixel coordinates ===
                x_norm = x_pixel / img_size
                y_norm = y_pixel / img_size
                norm_w = fixed_box_size_m / table_size_m
                norm_h = fixed_box_size_m / table_size_m
                xywh_norm_lines.append(f"{class_id} {x_norm:.6f} {y_norm:.6f} {norm_w:.6f} {norm_h:.6f}")
        
        else:
            print(f"  No detections after filtering")
            total_after_nms += 0
            xywh_norm_lines = []
    else:
        print(f"Image: {img_path.name} - No detections")
        xywh_norm_lines = []
    
    # Save the original rendered image (with all detections)
    rendered = result.plot()  # returns numpy array with boxes drawn
    output_img_path = output_img_dir / img_path.name
    cv2.imwrite(str(output_img_path), rendered)
    
    # Save filtered labels
    txt_path = pred_txt_dir / f"{img_path.stem}.txt"
    with open(txt_path, "w") as f:
        if xywh_norm_lines:
            f.write("\n".join(xywh_norm_lines))
        # If no detections, create empty file

print(f"\n✅ Processing complete!")
print(f"📊 Confidence-based Filtering Statistics:")
print(f"  - Total detections before filtering: {total_before_nms}")
print(f"  - Total detections after filtering: {total_after_nms}")
print(f"  - Reduction: {total_before_nms - total_after_nms} detections ({((total_before_nms - total_after_nms) / total_before_nms * 100):.1f}%)")
print(f"  - Average detections per image before: {total_before_nms / len(image_paths):.1f}")
print(f"  - Average detections per image after: {total_after_nms / len(image_paths):.1f}")
print(f"\n📁 Saved results:")
print(f"  - Images with original detections: {output_img_dir}")
print(f"  - Filtered labels: {pred_txt_dir}")

print(f"\n⚙️  Filtering Parameters used:")
print(f"  - IoU Threshold: {iou_threshold} (boxes with IoU ≥ this are considered overlapping)")
print(f"  - Confidence Threshold: {confidence_threshold} (minimum confidence to consider)")
print(f"\n💡 To adjust filtering:")
print(f"  - Lower IoU threshold (e.g., 0.5) = consider boxes overlapping with less overlap")
print(f"  - Higher IoU threshold (e.g., 0.8) = only consider boxes overlapping with more overlap")
print(f"  - Among overlapping boxes, always keep the one with highest confidence")