import numpy as np

# Translation vector from Left Mono to RGB (in meters, from provided data)
trans_left_to_rgb = np.array([-3.76055264, -0.05655098, -0.08987208])

# Convert to centimeters
rgb_origin_cm = trans_left_to_rgb * 100

# Print the origin of RGB camera relative to Left Mono
print("RGB Camera Origin (relative to Left Mono, in cm):")
print(f"X: {rgb_origin_cm[0]:.3f}, Y: {rgb_origin_cm[1]:.3f}, Z: {rgb_origin_cm[2]:.3f}")
