import cv2
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import io
from pathlib import Path
import os

class RealisticImageNoiseController:
    def __init__(self, image_path):
        """Initialize with image path"""
        self.original_image = cv2.imread(image_path)
        if self.original_image is None:
            raise ValueError(f"Could not read image from {image_path}")
        self.image_rgb = cv2.cvtColor(self.original_image, cv2.COLOR_BGR2RGB)
        self.height, self.width = self.image_rgb.shape[:2]
        
    # ========================= Illumination / Color =========================
    
    def adjust_exposure_ev(self, image, ev_stops=0.0):
        """
        Adjust exposure in EV stops
        Args:
            ev_stops (float): -2.0 to 2.0, exposure adjustment in stops
        """
        if abs(ev_stops) < 1e-6:
            return image
        gain = 2.0 ** ev_stops
        out = np.clip(image.astype(np.float32) * gain, 0, 255).astype(np.uint8)
        return out
    
    def adjust_white_balance(self, image, temp_shift=0.0, tint_shift=0.0):
        """
        Adjust white balance temperature and tint
        Args:
            temp_shift (float): -1.0 to 1.0, temperature shift (blue/yellow)
            tint_shift (float): -1.0 to 1.0, tint shift (green/magenta)
        """
        if abs(temp_shift) < 1e-6 and abs(tint_shift) < 1e-6:
            return image
            
        lab = cv2.cvtColor(image, cv2.COLOR_RGB2LAB).astype(np.float32)
        L, A, B = cv2.split(lab)
        
        # Apply temperature (yellow/blue axis)
        B += temp_shift * 20.0
        # Apply tint (green/magenta axis)
        A += tint_shift * 15.0
        
        lab = cv2.merge([L, A, B])
        out = cv2.cvtColor(np.clip(lab, 0, 255).astype(np.uint8), cv2.COLOR_LAB2RGB)
        return out
    
    def adjust_gamma(self, image, gamma=1.0):
        """
        Adjust gamma correction
        Args:
            gamma (float): 0.5 to 2.0, gamma value
        """
        if abs(gamma - 1.0) < 1e-6:
            return image
        inv_gamma = 1.0 / max(gamma, 1e-6)
        table = ((np.arange(256) / 255.0) ** inv_gamma * 255).astype(np.uint8)
        return cv2.LUT(image, table)
    
    def add_vignette(self, image, intensity=0.0):
        """
        Add vignetting effect (darker edges)
        Args:
            intensity (float): 0.0-1.0, vignette strength
        """
        if intensity <= 0:
            return image
            
        h, w = image.shape[:2]
        y, x = np.ogrid[:h, :w]
        cx, cy = w / 2, h / 2
        r = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
        mask = 1 - intensity * (r / r.max()) ** 2
        
        if image.ndim == 3:
            mask = mask[..., None]
        
        out = np.clip(image.astype(np.float32) * mask, 0, 255).astype(np.uint8)
        return out
    
    def add_uneven_illumination(self, image, intensity=0.0):
        """
        Add realistic uneven illumination patterns
        Args:
            intensity (float): 0.0-1.0, illumination variation strength
        """
        if intensity <= 0:
            return image
            
        h, w = image.shape[:2]
        
        # Create low-frequency illumination pattern
        noise_small = np.random.randn(h // 16 + 1, w // 16 + 1).astype(np.float32)
        illum = cv2.resize(noise_small, (w, h), interpolation=cv2.INTER_CUBIC)
        
        # Smooth the illumination
        sigma = max(h, w) / 10.0
        illum = cv2.GaussianBlur(illum, (0, 0), sigmaX=sigma, sigmaY=sigma)
        
        # Normalize and scale
        illum = (illum - illum.min()) / (illum.max() - illum.min() + 1e-6)
        illum = 0.85 + intensity * (illum - 0.5)
        
        if image.ndim == 3:
            illum = illum[..., None]
        
        out = np.clip(image.astype(np.float32) * illum, 0, 255).astype(np.uint8)
        return out
    
    # ============================== Optics ===============================
    
    def add_motion_blur(self, image, intensity=0.0, angle=None):
        """
        Add motion blur from camera/object movement
        Args:
            intensity (float): 0.0-1.0, blur intensity
            angle (float): blur direction in degrees (random if None)
        """
        if intensity <= 0:
            return image
            
        ksize = int(3 + intensity * 18)  # 3 to 21 pixels
        if ksize % 2 == 0:
            ksize += 1
            
        if angle is None:
            angle = np.random.uniform(-30, 30)
        
        # Create motion blur kernel
        kernel = np.zeros((ksize, ksize), np.float32)
        kernel[ksize // 2, :] = 1.0
        
        # Rotate kernel
        M = cv2.getRotationMatrix2D((ksize / 2 - 0.5, ksize / 2 - 0.5), angle, 1.0)
        kernel = cv2.warpAffine(kernel, M, (ksize, ksize))
        kernel = kernel / (kernel.sum() + 1e-6)
        
        return cv2.filter2D(image, -1, kernel)
    
    def add_defocus_blur(self, image, intensity=0.0):
        """
        Add defocus blur from poor focus
        Args:
            intensity (float): 0.0-1.0, defocus intensity
        """
        if intensity <= 0:
            return image
            
        radius = int(1 + intensity * 8)  # 1 to 9 pixels
        ksize = radius * 2 + 1
        return cv2.GaussianBlur(image, (ksize, ksize), 0)
    
    # ============================= Sensor ===============================
    
    def add_realistic_sensor_noise(self, image, shot_noise=0.0, read_noise=0.0):
        """
        Add realistic sensor noise (Poisson + Gaussian)
        Args:
            shot_noise (float): 0.0-1.0, photon shot noise intensity
            read_noise (float): 0.0-1.0, sensor read noise intensity
        """
        if shot_noise <= 0 and read_noise <= 0:
            return image
            
        # Convert to YCrCb to work primarily on luminance
        imgf = image.astype(np.float32) / 255.0
        ycrcb = cv2.cvtColor((imgf * 255).astype(np.uint8), cv2.COLOR_RGB2YCrCb).astype(np.float32) / 255.0
        Y = ycrcb[..., 0]
        
        # Add shot noise (Poisson)
        if shot_noise > 0:
            # Scale luminance to appropriate range for Poisson
            lambda_vals = np.clip(Y, 0, 1) * (shot_noise * 255.0)
            Y = np.clip(Y + (np.random.poisson(lambda_vals) / 255.0), 0.0, 1.0)
        
        # Add read noise (Gaussian)
        if read_noise > 0:
            Y = np.clip(Y + np.random.normal(0.0, read_noise / 255.0, Y.shape), 0.0, 1.0)
        
        ycrcb[..., 0] = Y
        out = cv2.cvtColor((np.clip(ycrcb, 0, 1) * 255).astype(np.uint8), cv2.COLOR_YCrCb2RGB)
        return out
    
    # ============================== Codec ===============================
    
    def add_compression_artifacts(self, image, intensity=0.0):
        """
        Add JPEG compression artifacts
        Args:
            intensity (float): 0.0-1.0, compression intensity
        """
        if intensity <= 0:
            return image
            
        # Map intensity to quality (95 to 50)
        quality = int(95 - intensity * 45)
        quality = max(quality, 50)
        
        # Compress and decompress
        buf = io.BytesIO()
        Image.fromarray(image).save(buf, format="JPEG", quality=quality)
        buf.seek(0)
        return np.array(Image.open(buf))
    
    # =================== Localization-hard extras =======================
    
    def add_soft_occlusion(self, image, intensity=0.0, count=1):
        """
        Add soft occlusion (simulates dust, water drops, etc.)
        Args:
            intensity (float): 0.0-1.0, occlusion intensity
            count (int): number of occlusion spots
        """
        if intensity <= 0 or count <= 0:
            return image
            
        h, w = image.shape[:2]
        out = image.copy().astype(np.float32)
        
        for _ in range(count):
            # Random occlusion size and position
            area = np.random.uniform(0.02, intensity * 0.15) * (h * w)
            aspect_ratio = np.random.uniform(0.6, 1.8)
            oh = int(np.sqrt(area / aspect_ratio))
            ow = int(aspect_ratio * oh)
            
            cx = np.random.randint(0, w)
            cy = np.random.randint(0, h)
            
            # Create soft mask
            mask = np.zeros((h, w), np.uint8)
            if np.random.rand() < 0.5:
                # Elliptical occlusion
                cv2.ellipse(mask, (cx, cy), (ow // 2, oh // 2),
                           angle=np.random.uniform(0, 180), startAngle=0, endAngle=360,
                           color=255, thickness=-1)
            else:
                # Rectangular occlusion
                box = cv2.boxPoints(((cx, cy), (ow, oh), np.random.uniform(0, 180)))
                box = np.int32(box)
                cv2.fillConvexPoly(mask, box, 255)
            
            # Soften edges
            softness = max(3, int(25 * intensity) | 1)
            mask = cv2.GaussianBlur(mask, (softness, softness), softness * 0.4)
            mask = mask.astype(np.float32) / 255.0
            mask = mask[..., None]
            
            # Apply occlusion (dark or bright)
            if np.random.rand() < 0.3:
                base_color = np.array([np.random.randint(60, 110)] * 3, np.float32)  # Bright
            else:
                base_color = np.array([np.random.randint(10, 40)] * 3, np.float32)   # Dark
            
            alpha = 0.7 * intensity
            out = out * (1.0 - alpha * mask) + base_color * (alpha * mask)
        
        return np.clip(out, 0, 255).astype(np.uint8)
    
    def add_shadow_band(self, image, intensity=0.0, width_factor=0.25, angle=35.0):
        """
        Add shadow band (simulates structural shadows)
        Args:
            intensity (float): 0.0-1.0, shadow intensity
            width_factor (float): 0.1-0.4, shadow width relative to image
            angle (float): shadow angle in degrees
        """
        if intensity <= 0:
            return image
            
        h, w = image.shape[:2]
        band_width = int(max(h, w) * width_factor)
        
        # Create shadow mask
        mask = np.zeros((h, w), np.float32)
        center = (w // 2 + np.random.randint(-w // 4, w // 4),
                 h // 2 + np.random.randint(-h // 4, h // 4))
        
        # Create rotated rectangle
        angle_variation = np.random.uniform(-15, 15)
        final_angle = angle + angle_variation
        box = cv2.boxPoints((center, (band_width * 3, band_width), final_angle))
        box = np.int32(box)
        cv2.fillConvexPoly(mask, box, 1.0)
        
        # Soften shadow edges
        mask = cv2.GaussianBlur(mask, (0, 0), sigmaX=band_width * 0.35)
        mask = (mask - mask.min()) / (mask.max() - mask.min() + 1e-6)
        
        # Apply shadow
        out = image.astype(np.float32)
        shadow_factor = 1.0 - 0.8 * intensity * mask[..., None]
        out *= shadow_factor
        
        return np.clip(out, 0, 255).astype(np.uint8)
    
    def add_glare_bloom(self, image, intensity=0.0, spots=1):
        """
        Add lens flare and bloom effects
        Args:
            intensity (float): 0.0-1.0, glare intensity
            spots (int): number of glare spots
        """
        if intensity <= 0 or spots <= 0:
            return image
            
        h, w = image.shape[:2]
        out = image.astype(np.float32)
        glare = np.zeros_like(out, np.float32)
        
        for _ in range(spots):
            # Random glare position and size
            cx = np.random.randint(int(0.1 * w), int(0.9 * w))
            cy = np.random.randint(int(0.1 * h), int(0.9 * h))
            radius = np.random.randint(int(0.05 * min(h, w)), int(0.18 * min(h, w)))
            
            # Create circular glare mask
            mask = np.zeros((h, w), np.float32)
            cv2.circle(mask, (cx, cy), radius, 1.0, -1)
            mask = cv2.GaussianBlur(mask, (0, 0), sigmaX=radius * 0.8)
            mask = (mask - mask.min()) / (mask.max() - mask.min() + 1e-6)
            
            glare += (mask[..., None] * 255.0)
        
        # Add brightness and reduce saturation
        out = np.clip(out + intensity * glare, 0, 255)
        out = cv2.cvtColor(out.astype(np.uint8), cv2.COLOR_RGB2HSV).astype(np.float32)
        out[..., 1] *= (1.0 - 0.15 * intensity)  # Reduce saturation
        out = cv2.cvtColor(np.clip(out, 0, 255).astype(np.uint8), cv2.COLOR_HSV2RGB)
        
        return out
    
    # ========================= Main Generation Method ======================
    
    def generate_realistic_noisy_image(self,
                                     # Exposure and color
                                     exposure_variation=0.0,
                                     white_balance_shift=0.0,
                                     gamma_variation=0.0,
                                     
                                     # Illumination
                                     uneven_illumination=0.0,
                                     vignette=0.0,
                                     
                                     # Optics
                                     motion_blur=0.0,
                                     defocus_blur=0.0,
                                     motion_angle=None,
                                     
                                     # Sensor
                                     shot_noise=0.0,
                                     read_noise=0.0,
                                     
                                     # Compression
                                     compression=0.0,
                                     
                                     # Localization challenges
                                     soft_occlusion=0.0,
                                     occlusion_count=1,
                                     shadow_band=0.0,
                                     shadow_width=0.25,
                                     shadow_angle=35.0,
                                     glare_bloom=0.0,
                                     glare_spots=1,
                                     
                                     show_comparison=False):
        """
        Generate realistic noisy image optimized for challenging object localization
        
        Args:
            exposure_variation (float): 0.0-1.0, exposure EV variation
            white_balance_shift (float): 0.0-1.0, color temperature shift
            gamma_variation (float): 0.0-1.0, gamma correction variation
            uneven_illumination (float): 0.0-1.0, lighting non-uniformity
            vignette (float): 0.0-1.0, edge darkening
            motion_blur (float): 0.0-1.0, movement blur
            defocus_blur (float): 0.0-1.0, focus blur
            motion_angle (float): blur direction (random if None)
            shot_noise (float): 0.0-1.0, photon noise
            read_noise (float): 0.0-1.0, sensor noise
            compression (float): 0.0-1.0, JPEG artifacts
            soft_occlusion (float): 0.0-1.0, partial occlusion
            occlusion_count (int): number of occlusion spots
            shadow_band (float): 0.0-1.0, structural shadows
            shadow_width (float): 0.1-0.4, shadow width factor
            shadow_angle (float): shadow angle in degrees
            glare_bloom (float): 0.0-1.0, lens flare intensity
            glare_spots (int): number of glare spots
            show_comparison (bool): display before/after
        
        Returns:
            numpy.ndarray: The processed image
        """
        
        # Start with original image
        result_image = self.image_rgb.copy()
        
        # 1) Illumination and color adjustments
        if exposure_variation > 0:
            ev_change = np.random.uniform(-1.0, 1.0) * exposure_variation * 2.0
            result_image = self.adjust_exposure_ev(result_image, ev_change)
        
        if white_balance_shift > 0:
            temp_shift = np.random.uniform(-1.0, 1.0) * white_balance_shift
            tint_shift = np.random.uniform(-1.0, 1.0) * white_balance_shift * 0.5
            result_image = self.adjust_white_balance(result_image, temp_shift, tint_shift)
        
        if gamma_variation > 0:
            gamma = 1.0 + np.random.uniform(-1.0, 1.0) * gamma_variation * 0.3
            result_image = self.adjust_gamma(result_image, gamma)
        
        result_image = self.add_uneven_illumination(result_image, uneven_illumination)
        result_image = self.add_vignette(result_image, vignette)
        
        # 2) Optical effects
        if motion_blur > 0:
            result_image = self.add_motion_blur(result_image, motion_blur, motion_angle)
        if defocus_blur > 0:
            result_image = self.add_defocus_blur(result_image, defocus_blur)
        
        # 3) Localization-challenging effects (applied strategically)
        if shadow_band > 0:
            result_image = self.add_shadow_band(result_image, shadow_band, shadow_width, shadow_angle)
        
        if soft_occlusion > 0:
            result_image = self.add_soft_occlusion(result_image, soft_occlusion, occlusion_count)
        
        if glare_bloom > 0:
            result_image = self.add_glare_bloom(result_image, glare_bloom, glare_spots)
        
        # 4) Sensor noise
        result_image = self.add_realistic_sensor_noise(result_image, shot_noise, read_noise)
        
        # 5) Compression (last step)
        result_image = self.add_compression_artifacts(result_image, compression)
        
        if show_comparison:
            self._show_comparison(result_image)
        
        return result_image
    
    def _show_comparison(self, noisy_image):
        """Show before and after comparison"""
        plt.figure(figsize=(12, 6))
        
        plt.subplot(1, 2, 1)
        plt.imshow(self.image_rgb)
        plt.title('Original Image')
        plt.axis('off')
        
        plt.subplot(1, 2, 2)
        plt.imshow(noisy_image)
        plt.title('Processed Image')
        plt.axis('off')
        
        plt.tight_layout()
        plt.show()
    
    def save_noisy_image(self, noisy_image, output_path):
        """Save the processed image"""
        # Convert RGB back to BGR for OpenCV
        noisy_image_bgr = cv2.cvtColor(noisy_image, cv2.COLOR_RGB2BGR)
        cv2.imwrite(output_path, noisy_image_bgr)
        print(f"Processed image saved to: {output_path}")


# ========================= Predefined Realistic Configurations =========================

# ========================= Three-Level Progressive Noise Configurations =========================

# LEVEL 1: EASY - Minimal realistic noise that slightly challenges detection
easy_noise_config = {
    'exposure_variation': 0.15,         # Very light exposure variation
    'white_balance_shift': 0.1,         # Minimal color temperature shift
    'gamma_variation': 0.1,             # Slight gamma variation
    'uneven_illumination': 0.2,         # Light illumination variation
    'vignette': 0.1,                    # Minimal vignetting
    'motion_blur': 0.1,                 # Very slight motion blur
    'defocus_blur': 0.0,                # No defocus blur
    'shot_noise': 0.05,                 # Very low photon noise
    'read_noise': 0.1,                  # Low sensor noise
    'compression': 0.15,                # Light JPEG compression
    'soft_occlusion': 0.05,             # Minimal occlusion (very light dust)
    'occlusion_count': 1,               # Single small occlusion
    'shadow_band': 0.1,                 # Very light shadows
    'shadow_width': 0.15,               # Narrow shadows
    'shadow_angle': 25.0,               # Moderate shadow angle
    'glare_bloom': 0.0,                 # No glare effects
    'glare_spots': 0,                   
    'show_comparison': False
}

# LEVEL 2: MEDIUM - Moderate noise that creates noticeable challenges
medium_noise_config = {
    'exposure_variation': 0.35,         # Moderate exposure issues
    'white_balance_shift': 0.25,        # Noticeable color shifts
    'gamma_variation': 0.18,            # Moderate gamma issues
    'uneven_illumination': 0.45,        # Clear lighting variation
    'vignette': 0.25,                   # Noticeable vignetting
    'motion_blur': 0.25,                # Moderate motion blur
    'defocus_blur': 0.1,                # Light defocus blur
    'shot_noise': 0.2,                  # Moderate photon noise
    'read_noise': 0.25,                 # Moderate sensor noise
    'compression': 0.3,                 # Noticeable JPEG artifacts
    'soft_occlusion': 0.15,             # Light to moderate occlusion
    'occlusion_count': 2,               # Two occlusion spots
    'shadow_band': 0.4,                 # Clear shadow bands
    'shadow_width': 0.25,               # Medium shadow width
    'shadow_angle': 35.0,               # Standard shadow angle
    'glare_bloom': 0.15,                # Light glare effects
    'glare_spots': 1,                   # Single glare spot
    'show_comparison': False
}

# LEVEL 3: HARD - Strong noise that significantly challenges localization
hard_noise_config = {
    'exposure_variation': 0.6,          # Strong exposure variation
    'white_balance_shift': 0.45,        # Strong color temperature issues
    'gamma_variation': 0.3,             # Significant gamma problems
    'uneven_illumination': 0.7,         # Strong lighting variation
    'vignette': 0.4,                    # Heavy vignetting
    'motion_blur': 0.4,                 # Strong motion blur
    'defocus_blur': 0.2,                # Moderate defocus blur
    'shot_noise': 0.4,                  # High photon noise
    'read_noise': 0.45,                 # High sensor noise
    'compression': 0.45,                # Heavy JPEG compression
    'soft_occlusion': 0.3,              # Significant occlusion
    'occlusion_count': 3,               # Multiple occlusion spots
    'shadow_band': 0.65,                # Strong shadow bands
    'shadow_width': 0.35,               # Wide shadows
    'shadow_angle': 45.0,               # Diagonal shadows
    'glare_bloom': 0.3,                 # Noticeable glare effects
    'glare_spots': 2,                   # Multiple glare spots
    'show_comparison': False
}

# Configuration mapping for easy access
NOISE_LEVELS = {
    'easy': easy_noise_config,
    'medium': medium_noise_config,
    'hard': hard_noise_config
}

def get_noise_config(level='medium'):
    """
    Get noise configuration by difficulty level
    
    Args:
        level (str): 'easy', 'medium', or 'hard'
    
    Returns:
        dict: Configuration dictionary
    """
    if level not in NOISE_LEVELS:
        raise ValueError(f"Unknown noise level: {level}. Choose from {list(NOISE_LEVELS.keys())}")
    
    return NOISE_LEVELS[level].copy()

# Updated processing function with three levels
def process_image_with_noise_level(image_path, output_path=None, level='medium', **custom_params):
    """
    Process image with three-level noise configurations
    
    Args:
        image_path: path to input image
        output_path: path to save output (optional)
        level: 'easy', 'medium', or 'hard'
        **custom_params: override any configuration parameters
    
    Returns:
        numpy.ndarray: processed image
    """
    
    controller = RealisticImageNoiseController(image_path)
    
    # Get configuration for the specified level
    config = get_noise_config(level)
    
    # Override with custom parameters
    config.update(custom_params)
    
    # Generate processed image
    processed_image = controller.generate_realistic_noisy_image(**config)
    
    # Save if output path provided
    if output_path:
        controller.save_noisy_image(processed_image, output_path)
    
    return processed_image

# Batch processing function for creating datasets at different difficulty levels
def create_noise_dataset(raw_image_dir, output_base_dir, levels=['easy', 'medium', 'hard'], 
                        num_images=500, file_extension='*.png'):
    """
    Create noise datasets at different difficulty levels
    
    Args:
        raw_image_dir (str): Directory containing raw images
        output_base_dir (str): Base directory for output
        levels (list): List of noise levels to generate
        num_images (int): Number of images to process
        file_extension (str): File extension pattern to match
    """
    import os
    from pathlib import Path
    
    # Get all raw image paths
    raw_image_paths = sorted(Path(raw_image_dir).glob(file_extension))
    
    if len(raw_image_paths) < num_images:
        print(f"Warning: Only {len(raw_image_paths)} images found, processing all available")
        num_images = len(raw_image_paths)
    
    for level in levels:
        # Create output directory for this level
        level_output_dir = os.path.join(output_base_dir, f'noise_level_{level}')
        os.makedirs(level_output_dir, exist_ok=True)
        
        print(f"Processing {num_images} images for {level} noise level...")
        
        for i in range(num_images):
            input_image_path = str(raw_image_paths[i])
            input_image_name = os.path.basename(input_image_path)
            
            # Add level prefix to filename
            name_parts = os.path.splitext(input_image_name)
            output_image_name = f"{level}_{name_parts[0]}{name_parts[1]}"
            output_image_path = os.path.join(level_output_dir, output_image_name)
            os.makedirs(os.path.dirname(output_image_path), exist_ok=True)

            try:
                # Process image with specified noise level
                process_image_with_noise_level(
                    image_path=input_image_path,
                    output_path=output_image_path,
                    level=level,
                    show_comparison=False
                )
                
                if (i + 1) % 50 == 0:
                    print(f"  Processed {i + 1}/{num_images} images for {level} level")
                    
            except Exception as e:
                print(f"Error processing {input_image_path}: {e}")
                continue
        
        print(f"Completed {level} noise level dataset")

# Example usage for your dissertation
if __name__ == "__main__":
    # Example paths - update these for your setup
    raw_image_dir = '/home/s2644572/robosuite/myCode/my_planning_app/logs/scene_images'
    noise_base_dir = '/home/s2644572/robosuite/myCode/my_planning_app/logs/scene_images/noise_levels'
    
    # Create datasets for all three noise levels
    create_noise_dataset(
        raw_image_dir=raw_image_dir,
        output_base_dir=noise_base_dir,
        levels=['easy', 'medium', 'hard'],
        num_images=500,
        file_extension='*.png'
    )
    
    print("All noise level datasets created successfully!")