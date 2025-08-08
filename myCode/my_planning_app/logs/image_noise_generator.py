import cv2
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import io

class ImageNoiseController:
    def __init__(self, image_path):
        """Initialize with image path"""
        self.original_image = cv2.imread(image_path)
        if self.original_image is None:
            raise ValueError(f"Could not read image from {image_path}")
        self.image_rgb = cv2.cvtColor(self.original_image, cv2.COLOR_BGR2RGB)
        self.height, self.width = self.image_rgb.shape[:2]
    
    def add_gaussian_noise(self, image, intensity=0.0):
        """
        Add Gaussian noise
        Args:
            intensity (float): 0.0-1.0, where 0 is no noise, 1 is maximum noise
        """
        if intensity <= 0:
            return image
        
        std = intensity * 50  # Scale to reasonable noise level
        noise = np.random.normal(0, std, image.shape).astype(np.int16)
        noisy_image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        return noisy_image
    
    def add_salt_pepper_noise(self, image, intensity=0.0):
        """
        Add salt and pepper noise
        Args:
            intensity (float): 0.0-1.0, where 0 is no noise, 1 is maximum noise
        """
        if intensity <= 0:
            return image
            
        noisy_image = image.copy()
        prob = intensity * 0.02  # Scale to reasonable probability
        
        # Salt noise (white pixels)
        salt_coords = np.random.random(image.shape[:2]) < prob/2
        noisy_image[salt_coords] = 255
        
        # Pepper noise (black pixels)
        pepper_coords = np.random.random(image.shape[:2]) < prob/2
        noisy_image[pepper_coords] = 0
        
        return noisy_image
    
    def add_speckle_noise(self, image, intensity=0.0):
        """
        Add speckle noise
        Args:
            intensity (float): 0.0-1.0, where 0 is no noise, 1 is maximum noise
        """
        if intensity <= 0:
            return image
            
        noise_factor = intensity * 0.3  # Scale to reasonable speckle level
        noise = np.random.randn(*image.shape) * noise_factor
        noisy_image = image + image * noise
        noisy_image = np.clip(noisy_image, 0, 255).astype(np.uint8)
        return noisy_image
    
    def add_uniform_noise(self, image, intensity=0.0):
        """
        Add uniform noise
        Args:
            intensity (float): 0.0-1.0, where 0 is no noise, 1 is maximum noise
        """
        if intensity <= 0:
            return image
            
        noise_range = intensity * 40  # Scale to reasonable range
        noise = np.random.uniform(-noise_range, noise_range, image.shape).astype(np.int16)
        noisy_image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        return noisy_image
    
    def add_blur(self, image, intensity=0.0):
        """
        Add Gaussian blur
        Args:
            intensity (float): 0.0-1.0, where 0 is no blur, 1 is maximum blur
        """
        if intensity <= 0:
            return image
            
        # Convert intensity to kernel size (odd numbers only)
        kernel_size = int(intensity * 10) * 2 + 1
        if kernel_size < 3:
            return image
            
        return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)
    
    def add_compression_artifacts(self, image, intensity=0.0):
        """
        Add JPEG compression artifacts
        Args:
            intensity (float): 0.0-1.0, where 0 is no compression, 1 is maximum compression
        """
        if intensity <= 0:
            return image
            
        # Convert intensity to quality (100 = no compression, lower = more compression)
        quality = int(100 - intensity * 50)  # Range: 100 to 50
        quality = max(quality, 10)  # Ensure minimum quality
        
        # Convert to PIL, compress, and convert back
        pil_image = Image.fromarray(image)
        buffer = io.BytesIO()
        pil_image.save(buffer, format='JPEG', quality=quality)
        buffer.seek(0)
        compressed_image = Image.open(buffer)
        return np.array(compressed_image)
    
    def add_brightness_variation(self, image, intensity=0.0):
        """
        Add random brightness variations
        Args:
            intensity (float): 0.0-1.0, where 0 is no variation, 1 is maximum variation
        """
        if intensity <= 0:
            return image
            
        variation_range = intensity * 30  # Scale to reasonable brightness range
        brightness_noise = np.random.uniform(-variation_range, variation_range, 
                                           image.shape[:2])
        
        if len(image.shape) == 3:
            brightness_noise = brightness_noise[:, :, np.newaxis]
            
        noisy_image = np.clip(image.astype(np.int16) + brightness_noise, 
                             0, 255).astype(np.uint8)
        return noisy_image
    
    def add_poisson_noise(self, image, intensity=0.0):
        """
        Add Poisson noise (photon noise)
        Args:
            intensity (float): 0.0-1.0, where 0 is no noise, 1 is maximum noise
        """
        if intensity <= 0:
            return image
            
        # Scale image to have appropriate lambda for Poisson distribution
        scaling_factor = (1.0 - intensity) * 100 + 1  # Higher intensity = lower scaling = more noise
        scaled_image = image / scaling_factor
        
        # Generate Poisson noise
        noisy_image = np.random.poisson(scaled_image) * scaling_factor
        noisy_image = np.clip(noisy_image, 0, 255).astype(np.uint8)
        return noisy_image
    
    def add_directional_lighting(self, image, intensity=0.0, light_angle=45, light_x=0.3, light_y=0.2):
        """
        Add directional lighting effects with shadows and illumination gradients
        Args:
            intensity (float): 0.0-1.0, where 0 is uniform lighting, 1 is strong directional
            light_angle (float): Light direction angle in degrees (0-360)
            light_x (float): Light source X position (0.0-1.0, relative to image width)
            light_y (float): Light source Y position (0.0-1.0, relative to image height)
        """
        if intensity <= 0:
            return image
            
        result_image = image.copy().astype(np.float32)
        
        # Create coordinate grids
        y_coords, x_coords = np.mgrid[0:self.height, 0:self.width]
        
        # Light source position
        light_pos_x = light_x * self.width
        light_pos_y = light_y * self.height
        
        # Calculate distance from each pixel to light source
        distance_from_light = np.sqrt((x_coords - light_pos_x)**2 + (y_coords - light_pos_y)**2)
        max_distance = np.sqrt(self.width**2 + self.height**2)
        
        # Create illumination falloff (inverse square law, but softened)
        illumination = 1.0 - (distance_from_light / max_distance) * intensity * 0.7
        illumination = np.clip(illumination, 0.3, 1.0)  # Prevent complete darkness
        
        # Create directional gradient based on angle
        angle_rad = np.radians(light_angle)
        direction_x = np.cos(angle_rad)
        direction_y = np.sin(angle_rad)
        
        # Normalize coordinates to [-1, 1]
        norm_x = (x_coords / self.width - 0.5) * 2
        norm_y = (y_coords / self.height - 0.5) * 2
        
        # Calculate directional component
        directional_component = (norm_x * direction_x + norm_y * direction_y)
        directional_gradient = 1.0 + directional_component * intensity * 0.4
        directional_gradient = np.clip(directional_gradient, 0.4, 1.6)
        
        # Combine illumination effects
        final_lighting = illumination * directional_gradient
        
        # Apply lighting to each channel
        if len(image.shape) == 3:
            final_lighting = final_lighting[:, :, np.newaxis]
        
        result_image = result_image * final_lighting
        result_image = np.clip(result_image, 0, 255).astype(np.uint8)
        
        return result_image
    
    def add_shadow_effects(self, image, intensity=0.0, shadow_angle=225, shadow_length=0.3):
        """
        Add shadow effects that simulate objects casting shadows
        Args:
            intensity (float): 0.0-1.0, shadow darkness
            shadow_angle (float): Shadow direction in degrees
            shadow_length (float): 0.0-1.0, relative shadow length
        """
        if intensity <= 0:
            return image
            
        result_image = image.copy()
        
        # Convert to HSV for better shadow control
        hsv_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2HSV).astype(np.float32)
        
        # Create shadow mask based on image brightness
        brightness = cv2.cvtColor(result_image, cv2.COLOR_RGB2GRAY).astype(np.float32) / 255.0
        
        # Detect potential shadow-casting areas (brighter regions)
        shadow_sources = brightness > 0.5
        
        # Create shadow offset based on angle
        angle_rad = np.radians(shadow_angle)
        shadow_offset_x = int(shadow_length * self.width * 0.1 * np.cos(angle_rad))
        shadow_offset_y = int(shadow_length * self.height * 0.1 * np.sin(angle_rad))
        
        # Create shadow map
        shadow_map = np.zeros_like(brightness)
        
        # Shift shadow sources to create shadow areas
        if shadow_offset_x != 0 or shadow_offset_y != 0:
            shifted_sources = np.roll(shadow_sources, shadow_offset_y, axis=0)
            shifted_sources = np.roll(shifted_sources, shadow_offset_x, axis=1)
            
            # Create gradient shadow effect
            kernel_size = max(3, int(shadow_length * 20))
            if kernel_size % 2 == 0:
                kernel_size += 1
            
            shadow_blur = cv2.GaussianBlur(shifted_sources.astype(np.float32), 
                                         (kernel_size, kernel_size), 0)
            shadow_map = shadow_blur * intensity * 0.6
        
        # Apply shadows by reducing brightness (V channel in HSV)
        hsv_image[:, :, 2] = hsv_image[:, :, 2] * (1.0 - shadow_map)
        hsv_image = np.clip(hsv_image, 0, 255)
        
        # Convert back to RGB
        result_image = cv2.cvtColor(hsv_image.astype(np.uint8), cv2.COLOR_HSV2RGB)
        
        return result_image
    
    def add_ambient_occlusion(self, image, intensity=0.0):
        """
        Add ambient occlusion effects (darker areas where surfaces meet)
        Args:
            intensity (float): 0.0-1.0, occlusion strength
        """
        if intensity <= 0:
            return image
            
        # Convert to grayscale to detect edges/corners
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        
        # Detect edges using Sobel operator
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        edge_magnitude = np.sqrt(sobelx**2 + sobely**2)
        
        # Normalize edge magnitude
        edge_magnitude = edge_magnitude / np.max(edge_magnitude)
        
        # Create occlusion map (stronger near edges)
        occlusion_map = edge_magnitude * intensity * 0.3
        
        # Blur the occlusion to make it more natural
        occlusion_map = cv2.GaussianBlur(occlusion_map, (15, 15), 0)
        
        # Apply occlusion by darkening the image
        result_image = image.copy().astype(np.float32)
        
        if len(image.shape) == 3:
            occlusion_map = occlusion_map[:, :, np.newaxis]
        
        result_image = result_image * (1.0 - occlusion_map)
        result_image = np.clip(result_image, 0, 255).astype(np.uint8)
        
        return result_image
    
    def generate_noisy_image(self, 
                           gaussian_noise=0.0,
                           salt_pepper_noise=0.0,
                           speckle_noise=0.0,
                           uniform_noise=0.0,
                           blur=0.0,
                           compression=0.0,
                           brightness_variation=0.0,
                           poisson_noise=0.0,
                           directional_lighting=0.0,
                           light_angle=45,
                           light_x=0.3,
                           light_y=0.2,
                           shadow_effects=0.0,
                           shadow_angle=225,
                           shadow_length=0.3,
                           ambient_occlusion=0.0,
                           show_comparison=False):
        """
        Generate a single noisy image with controlled noise levels and lighting effects
        
        Args:
            gaussian_noise (float): 0.0-1.0
            salt_pepper_noise (float): 0.0-1.0
            speckle_noise (float): 0.0-1.0
            uniform_noise (float): 0.0-1.0
            blur (float): 0.0-1.0
            compression (float): 0.0-1.0
            brightness_variation (float): 0.0-1.0
            poisson_noise (float): 0.0-1.0
            directional_lighting (float): 0.0-1.0, lighting non-uniformity
            light_angle (float): Light direction in degrees (0-360)
            light_x (float): Light source X position (0.0-1.0)
            light_y (float): Light source Y position (0.0-1.0)
            shadow_effects (float): 0.0-1.0, shadow intensity
            shadow_angle (float): Shadow direction in degrees
            shadow_length (float): 0.0-1.0, shadow length
            ambient_occlusion (float): 0.0-1.0, corner/edge darkening
            show_comparison (bool): Whether to show before/after comparison
        
        Returns:
            numpy.ndarray: The noisy image with lighting effects
        """
        
        # Start with original image
        result_image = self.image_rgb.copy()
        
        # Apply lighting effects first (they affect the base illumination)
        result_image = self.add_directional_lighting(result_image, directional_lighting, 
                                                   light_angle, light_x, light_y)
        result_image = self.add_shadow_effects(result_image, shadow_effects, 
                                             shadow_angle, shadow_length)
        result_image = self.add_ambient_occlusion(result_image, ambient_occlusion)
        
        # Then apply optical effects
        result_image = self.add_blur(result_image, blur)
        
        # Apply noise types
        result_image = self.add_gaussian_noise(result_image, gaussian_noise)
        result_image = self.add_salt_pepper_noise(result_image, salt_pepper_noise)
        result_image = self.add_speckle_noise(result_image, speckle_noise)
        result_image = self.add_uniform_noise(result_image, uniform_noise)
        result_image = self.add_poisson_noise(result_image, poisson_noise)
        result_image = self.add_brightness_variation(result_image, brightness_variation)
        
        # Apply compression last (simulates camera/storage effects)
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
        plt.title('Noisy Image')
        plt.axis('off')
        
        plt.tight_layout()
        plt.show()
    
    def save_noisy_image(self, noisy_image, output_path):
        """Save the noisy image"""
        # Convert RGB back to BGR for OpenCV
        noisy_image_bgr = cv2.cvtColor(noisy_image, cv2.COLOR_RGB2BGR)
        cv2.imwrite(output_path, noisy_image_bgr)
        print(f"Noisy image saved to: {output_path}")


# Example usage function
def process_image_with_controlled_noise(image_path, output_path=None, **noise_params):
    """
    Process image with controlled noise parameters
    
    Example usage:
        result = process_image_with_controlled_noise(
            'input.png',
            'output.png',
            gaussian_noise=0.3,
            blur=0.2,
            brightness_variation=0.1
        )
    """
    
    controller = ImageNoiseController(image_path)
    
    # Generate noisy image
    noisy_image = controller.generate_noisy_image(**noise_params, show_comparison=True)
    
    # Save if output path provided
    if output_path:
        controller.save_noisy_image(noisy_image, output_path)
    
    return noisy_image


# Example usage:
if __name__ == "__main__":

    for scene_idx in ["01", "02", "03", "04", "05"]:
        # Replace with your image path
        input_image_path = f"/home/jingyang/robosuite/myCode/my_planning_app/logs/scene_{scene_idx}/env_and_func_rendered.png"
        output_image_dir = f"/home/jingyang/robosuite/myCode/my_planning_app/logs/scene_{scene_idx}/"

        try:
            # Example 1: Realistic lighting from top-left
            realistic_lighting_result = process_image_with_controlled_noise(
                input_image_path,
                output_image_dir + "env_and_func_noised_rendered.png",
                directional_lighting=0.4,      # Moderate directional lighting
                light_angle=135,               # Light from top-left (135°)
                light_x=0.2,                   # Light source at 20% from left
                light_y=0.2,                   # Light source at 20% from top
                shadow_effects=0.3,            # Medium shadows
                shadow_angle=315,              # Shadows opposite to light (315°)
                shadow_length=0.4,             # Medium shadow length
                ambient_occlusion=0.2,         # Light corner darkening
                gaussian_noise=0.1,            # Light noise
                blur=0.05                      # Minimal blur
            )
            
            # Quick reference for lighting parameters:
            print("\n=== Lighting Parameters Guide ===")
            print("directional_lighting: 0.0-1.0 (0=uniform, 1=strong directional)")
            print("light_angle: 0-360° (0=right, 90=down, 180=left, 270=up)")
            print("light_x/light_y: 0.0-1.0 (light source position)")
            print("shadow_effects: 0.0-1.0 (shadow darkness)")
            print("shadow_angle: 0-360° (usually opposite to light_angle)")
            print("ambient_occlusion: 0.0-1.0 (corner/edge darkening)")
            
        except Exception as e:
            print(f"Error: {e}")
            print("Make sure to update the image path to your actual file location.")