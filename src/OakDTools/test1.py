import cv2
import depthai as dai

def create_pipeline():
    """Create and configure the DepthAI pipeline for RGB camera only"""
    pipeline = dai.Pipeline()
    
    # Create color camera node
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    
    # Create output
    rgb_out = pipeline.create(dai.node.XLinkOut)
    rgb_out.setStreamName("rgb")
    cam_rgb.preview.link(rgb_out.input)
    
    return pipeline

def main():
    """Main function to run the OAK-D RGB camera"""
    try:
        # Create pipeline
        pipeline = create_pipeline()
        
        # Connect to device and start pipeline
        with dai.Device(pipeline) as device:
            print("Connected to OAK-D camera")
            
            # Get output queue
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            while True:
                # Get frame from queue
                in_rgb = q_rgb.get()
                rgb_frame = in_rgb.getCvFrame()
                
                # Display the frame
                cv2.imshow("OAK-D RGB Camera", rgb_frame)
                
                # Break loop on 'q' key press
                if cv2.waitKey(1) == ord('q'):
                    break
                    
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure your OAK-D camera is connected and DepthAI is installed")
    
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
