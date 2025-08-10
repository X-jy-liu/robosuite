from ultralytics import YOLO
import cv2
from pathlib import Path

def run_yolo_inference(model_path, test_img_dir, output_base_dir, num_images=500, 
                       img_size=512, table_size_m=0.8, fixed_box_size_m=0.05):
    """
    Run YOLO inference on test images and save predictions with calibrated coordinates
    
    Args:
        model_path (str): Path to the trained YOLO model (.pt file)
        test_img_dir (str or Path): Directory containing test images
        output_base_dir (str or Path): Base directory for output files
        num_images (int): Maximum number of images to process
        img_size (int): Image size in pixels (assuming square images)
        table_size_m (float): Size of the tabletop in meters
        fixed_box_size_m (float): Fixed bounding box size in meters
    
    Returns:
        tuple: (output_img_dir, pred_txt_dir) - paths to saved outputs
    """
    
    # Convert to Path objects
    test_img_dir = Path(test_img_dir)
    output_base_dir = Path(output_base_dir)
    
    # Create output directories
    output_img_dir = output_base_dir / "images"
    pred_txt_dir = output_base_dir / "labels"
    
    output_img_dir.mkdir(parents=True, exist_ok=True)
    pred_txt_dir.mkdir(parents=True, exist_ok=True)
    
    # Load model
    print(f"Loading YOLO model from: {model_path}")
    model = YOLO(model_path)
    
    # Get image paths
    image_paths = sorted([p for p in test_img_dir.glob("*.jpg")] + 
                        [p for p in test_img_dir.glob("*.png")])[:num_images]
    
    if not image_paths:
        raise ValueError(f"No images found in {test_img_dir}")
    
    print(f"Found {len(image_paths)} images to process")
    
    # Calculate conversion parameters
    meters_per_pixel = table_size_m / img_size
    
    # Process each image
    processed_count = 0
    for i, img_path in enumerate(image_paths):
        try:
            # Run inference
            result = model(img_path)[0]
            
            # Save rendered image with boxes
            rendered = result.plot()  # returns numpy array with boxes drawn
            output_img_path = output_img_dir / img_path.name
            cv2.imwrite(str(output_img_path), rendered)
            
            # Extract prediction data
            if result.boxes is not None and len(result.boxes) > 0:
                boxes = result.boxes.xywh.cpu().numpy()  # x_center, y_center, width, height (in pixels)
                classes = result.boxes.cls.cpu().numpy()
                
                xywh_norm_lines = []
                for xywh, cls in zip(boxes, classes):
                    x_pixel, y_pixel, _, _ = xywh
                    class_id = int(cls)
                    
                    # Normalize pixel coordinates
                    x_norm = x_pixel / img_size
                    y_norm = y_pixel / img_size
                    norm_w = fixed_box_size_m / table_size_m
                    norm_h = fixed_box_size_m / table_size_m
                    
                    xywh_norm_lines.append(f"{class_id} {x_norm:.6f} {y_norm:.6f} {norm_w:.6f} {norm_h:.6f}")
            else:
                # No detections found
                xywh_norm_lines = []
            
            # Save calibrated labels
            txt_path = pred_txt_dir / f"{img_path.stem}.txt"
            with open(txt_path, "w") as f:
                if xywh_norm_lines:
                    f.write("\n".join(xywh_norm_lines))
                # If no detections, create empty file
            
            processed_count += 1
            
            # Progress update
            if (i + 1) % 50 == 0:
                print(f"Processed {i + 1}/{len(image_paths)} images...")
                
        except Exception as e:
            print(f"Error processing {img_path}: {e}")
            continue
    
    print(f"✅ Successfully processed {processed_count} images")
    print(f" - Output Images: {output_img_dir}")
    print(f" - Prediction Labels: {pred_txt_dir}")
    
    return output_img_dir, pred_txt_dir

def main():
    """
    Main function with example usage
    """
    # Configuration parameters
    model_path = "/home/s2644572/robosuite/myCode/yolo_perception/runs/detect/mini_tabletop_train_1/weights/best.pt"
    
    # Example for different noise levels
    noise_levels = ["easy", "medium", "hard"]
    base_test_dir = "/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/noise_levels"
    base_output_dir = "/home/s2644572/robosuite/myCode/yolo_perception/data/tabletop/test/inference_from_mini_train_3_levels_realistic_noise"
    import os
    os.makedirs(base_output_dir, exist_ok=True)
    for noise_level in noise_levels:
        print(f"\n{'='*50}")
        print(f"Processing {noise_level.upper()} noise level")
        print(f"{'='*50}")

        test_img_dir = Path(base_test_dir) / f"noise_level_{noise_level}"
        output_dir = Path(base_output_dir) / noise_level
        
        try:
            run_yolo_inference(
                model_path=model_path,
                test_img_dir=test_img_dir,
                output_base_dir=output_dir,
                num_images=500,
                img_size=512,
                table_size_m=0.8,
                fixed_box_size_m=0.05
            )
        except Exception as e:
            print(f"Error processing {noise_level} level: {e}")
            continue
    
    print(f"\n✅ All noise levels processed successfully!")

# Alternative function for single directory processing
def run_single_inference(model_path, test_img_dir, output_base_dir, **kwargs):
    """
    Simplified function for processing a single directory
    
    Args:
        model_path (str): Path to YOLO model
        test_img_dir (str): Input image directory
        output_base_dir (str): Output directory
        **kwargs: Additional parameters for run_yolo_inference
    """
    return run_yolo_inference(model_path, test_img_dir, output_base_dir, **kwargs)

if __name__ == "__main__":
    # Example usage - uncomment to run
    main()