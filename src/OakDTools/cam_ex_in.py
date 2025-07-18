import depthai as dai
import numpy as np

def print_extrinsics(extrinsics, camera1, camera2):
    """Helper function to print extrinsics matrix and translation vector."""
    # Convert list to NumPy array for proper slicing
    extrinsics = np.array(extrinsics)
    print(f"\nExtrinsics from {camera1} to {camera2}:")
    print("Rotation Matrix:")
    print(extrinsics[:3, :3])
    print("Translation Vector (meters):")
    print(extrinsics[:3, 3])

# Create a pipeline
pipeline = dai.Pipeline()

# Create a device and connect to it
with dai.Device(pipeline) as device:
    # Get the calibration handler
    calib_data = device.readCalibration()

    # Retrieve extrinsics between different camera pairs
    # Extrinsics from left mono to right mono
    extrinsics_left_to_right = calib_data.getCameraExtrinsics(
        dai.CameraBoardSocket.LEFT, dai.CameraBoardSocket.RIGHT
    )
    print_extrinsics(extrinsics_left_to_right, "Left Mono", "Right Mono")

    # Extrinsics from left mono to RGB
    extrinsics_left_to_rgb = calib_data.getCameraExtrinsics(
        dai.CameraBoardSocket.LEFT, dai.CameraBoardSocket.RGB
    )
    print_extrinsics(extrinsics_left_to_rgb, "Left Mono", "RGB")

    # Extrinsics from right mono to RGB
    extrinsics_right_to_rgb = calib_data.getCameraExtrinsics(
        dai.CameraBoardSocket.RIGHT, dai.CameraBoardSocket.RGB
    )
    print_extrinsics(extrinsics_right_to_rgb, "Right Mono", "RGB")

    # Optionally, save extrinsics to a file
    np.savez(
        'oakd_extrinsics.npz',
        left_to_right=np.array(extrinsics_left_to_right),
        left_to_rgb=np.array(extrinsics_left_to_rgb),
        right_to_rgb=np.array(extrinsics_right_to_rgb)
    )
    print("\nExtrinsics saved to 'oakd_extrinsics.npz'")
