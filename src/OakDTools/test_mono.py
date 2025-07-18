import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

# Define left and right mono cameras
mono_left = pipeline.create(dai.node.MonoCamera)
mono_right = pipeline.create(dai.node.MonoCamera)

# Define output streams for both cameras
xout_left = pipeline.create(dai.node.XLinkOut)
xout_right = pipeline.create(dai.node.XLinkOut)

# Set stream names
xout_left.setStreamName("left")
xout_right.setStreamName("right")

# Configure camera properties
mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
mono_left.setFps(30)
mono_right.setFps(30)

# Link cameras to output streams
mono_left.out.link(xout_left.input)
mono_right.out.link(xout_right.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    print("Connected cameras:", device.getConnectedCameras())
    print("USB speed:", device.getUsbSpeed().name)

    # Get output queues
    queue_left = device.getOutputQueue(name="left", maxSize=4, blocking=False)
    queue_right = device.getOutputQueue(name="right", maxSize=4, blocking=False)

    while True:
        # Get frames from left and right cameras
        in_left = queue_left.get()
        in_right = queue_right.get()

        # Convert frames to OpenCV format
        frame_left = in_left.getCvFrame()
        frame_right = in_right.getCvFrame()

        # Display frames
        cv2.imshow("Left Mono Camera", frame_left)
        cv2.imshow("Right Mono Camera", frame_right)

        # Exit on 'q' key press
        if cv2.waitKey(1) == ord("q"):
            break

# Clean up
cv2.destroyAllWindows()
