#!/usr/bin/env python3

##!/usr/bin/env python3

from operator import truediv
from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import os
import signal
import open3d
'''
Spatial Tiny-yolo 
  Performs inference on RGB camera and retrieves spatial location coordinates: x,y,z relative to the center of depth map.
  Can be used for tiny-yolo-v3 or tiny-yolo-v4 networks
'''
global device
global inDet
global count
global DetectionOutputRequired

DetectionOutputRequired = False 
#IMAGELOGGING = "false" 
IMAGELOGGING = "true" 
Initialisation_time_start = time.monotonic()

# Get argument first
nnBlobPath = str((Path(__file__).parent / Path('yolov5_openvino_2021.4_6shave (3).blob')).resolve().absolute())
if len(sys.argv) > 1:
    nnBlobPath = sys.argv[1]

if not Path(nnBlobPath).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

# yolo v3/4 label texts
labelMap = [
    "cotton"
]

syncNN = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.createColorCamera()
spatialDetectionNetwork = pipeline.createYoloSpatialDetectionNetwork()
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()
manip=pipeline.create(dai.node.ImageManip)
manipOut=pipeline.createXLinkOut()
manipOut.setStreamName("manip")

xoutRgb = pipeline.createXLinkOut()
xoutNN = pipeline.createXLinkOut()
xoutBoundingBoxDepthMapping = pipeline.createXLinkOut()
xoutDepth = pipeline.createXLinkOut()

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
xoutDepth.setStreamName("depth")

# Properties
manip.initialConfig.setResizeThumbnail(416,416)
camRgb.setPreviewSize(1920,1080)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# setting node configs
lrcheck = False
subpixel = False
stereo.initialConfig.setConfidenceThreshold(255)
stereo.setLeftRightCheck(lrcheck)
stereo.setSubpixel(subpixel)
stereo.setExtendedDisparity(True)

spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

# Yolo specific parameters
spatialDetectionNetwork.setNumClasses(1)
spatialDetectionNetwork.setCoordinateSize(4)
spatialDetectionNetwork.setAnchors(np.array([10,13, 16,30, 33,23, 30,61, 62,45, 59,119, 116,90, 156,198, 373,326]))
spatialDetectionNetwork.setAnchorMasks({"side52": np.array([0,1,2]), "side26": np.array([3,4,5]), "side13": np.array([6,7,8]) })
spatialDetectionNetwork.setIouThreshold(0.5)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(manip.inputImage)
manip.out.link(spatialDetectionNetwork.input)
if syncNN:
    spatialDetectionNetwork.passthrough.link(xoutRgb.input)
else:
    camRgb.preview.link(xoutRgb.input)
manip.out.link(manipOut.input)
spatialDetectionNetwork.out.link(xoutNN.input)
spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

# Connect to device and start pipeline

def WaitOnSignal():
    global DetectionOutputRequired 
    print("CottonDetect WaitOnSignal : Waiting for SIGUSR1")
    sig = signal.sigwait([signal.SIGTERM, signal.SIGUSR1, signal.SIGHUP])
    print("CottonDetect WaitOnSignal : Returned after receiving SIGUSR1")
    if (sig == signal.SIGUSR1):
            #PrintCottonDetailsTxtFile()
            print("CottonDetect : Got SIGUSR1")
            #DetectCotton()
            DetectionOutputRequired = True
            #continue
    if (sig == signal.SIGHUP):
            #self.load_configuration()
            print("CottonDetect : Got SIGHUP")
            #continue
    
    if (sig == signal.SIGTERM):
            #logging.info('Got SIGTERM, exiting
            print("CottonDetect : Got SIGTERM")
            #self.flush_thread.join()
            #continue

import psutil
def find_procs_by_name(name):
    "Return a list of processes matching 'name'."                                             
    ls = []
    for p in psutil.process_iter(attrs=["name", "exe", "cmdline"]):
        if name == p.info['name'] or \
                p.info['exe'] and os.path.basename(p.info['exe']) == name or \
                p.info['cmdline'] and p.info['cmdline'][0] == name:
            ls.append(p)
    return ls

    
#signal.signal(signal.SIGUSR1,Detect_cotton)

def main():
    global device
    global count
    global inDet
    global DetectionOutputRequired 
    count = 0 
    ParentProcessID = os.getppid()
    ## TODO pass down parent PID or find a reliable
    #Find Parent Process ID or find it by name  

    with dai.Device(pipeline) as device:

        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        xoutBoundingBoxDepthMappingQueue = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
        depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        qman = device.getOutputQueue(name="manip", maxSize=4, blocking=False)
        Initialisation_time_finish = time.monotonic()
        Initialisation_time= Initialisation_time_finish - Initialisation_time_start
        
        PCLqueue = device.getOutputQueue("pcl", maxSize=8, blocking=False)
        print(f"Initialisation_time:{Initialisation_time}\n")
    
        startTime = time.monotonic()
        counter = 0
        ImageCount = 0
        fps = 0
        color = (255, 255, 255)
        # System Ready for Detection after initialisation
        # Send Signal to parent the ready Status & Then wait for Command from parent
        os.kill(ParentProcessID, signal.SIGUSR2)

        while True:
            if (IMAGELOGGING == "true"): 
               inPreview = previewQueue.get()
               frame = inPreview.getCvFrame()
               cv2.imwrite( str(ImageCount) + '.jpg', frame) 
               ImageCount = ImageCount + 1

            if (DetectionOutputRequired):
               DetectionOutputRequired = False 
               inPreview = previewQueue.get()
               inDet = detectionNNQueue.get()
               depth = depthQueue.get()

               frame = inPreview.getCvFrame()
               depthFrame = depth.getFrame()
               depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
               depthFrameColor = cv2.equalizeHist(depthFrameColor)
               depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

               counter+=1
               current_time = time.monotonic()
               if (current_time - startTime) > 1 :
                   fps = counter / (current_time - startTime)
                   counter = 0
                   startTime = current_time
                   detections = inDet.detections
                   if len(detections) != 0:
                       boundingBoxMapping = xoutBoundingBoxDepthMappingQueue.get()
                       roiDatas = boundingBoxMapping.getConfigData()

                       for roiData in roiDatas:
                           roi = roiData.roi
                           roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
                           topLeft = roi.topLeft()
                           bottomRight = roi.bottomRight()
                           xmin = int(topLeft.x)
                           ymin = int(topLeft.y)
                           xmax = int(bottomRight.x)
                           ymax = int(bottomRight.y)

                           cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)


                   # If the frame is available, draw bounding boxes on it and show the frame
                   height = frame.shape[0]
                   width  = frame.shape[1]
                   txt=""
                   for detection in detections:
                       # Denormalize bounding box
                       x1 = int(detection.xmin * width)
                       x2 = int(detection.xmax * width)
                       y1 = int(detection.ymin * height)
                       y2 = int(detection.ymax * height)
                       try:
                           label = labelMap[detection.label]
                       except:
                           label = detection.label
                       cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                       #cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                       #cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                       #cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                       #cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                       cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)
                       t=detection
                       txt = txt + "636 0 " + str(round(float(t.spatialCoordinates.x) / 1000, 5)) + " " + str(round(float(t.spatialCoordinates.y) / 1000, 5)) + " " + str(round(float(t.spatialCoordinates.z) / 1000, 5)) + "\n"
                   
                   #pcd = open3d.geometry.PointCloud.create_from_depth_image(depthFrame)
                   pcl_data = np.array(PCDqueue.get().getFirstLayerFp16()).reshape(1, 3, resolution[1], resolution[0])
                   pcl_data = pcl_data.reshape(3, -1).T.astype(np.float64) / 1000.0
                   #pcl_converter.visualize_pcl(pcl_data, downsample=downsample_pcl)
                   open3d.io.write_point_cloud("DetectionOutput.pcd", pcl_data)


                   file2 = open(r".\cotton_details.txt", "w+")
                   file2.write(txt)
                   file2.close()
                   #cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
                   #cv2.imshow("depth", depthFrameColor)
                   #cv2.imshow("rgb", frame)
                   cv2.imwrite("DetectionOutput.jpg", frame)
                   #### Endof if (DetectionOutputsRequired)
               #if cv2.waitKey(1) == ord('q'):
               #    break
              #inDet = detectionNNQueue.get()
            WaitOnSignal()
            print(f"CottonDetect : Sending SIGUSR2 signal to ParentProcess at {time.ctime()} \n")
            #YanthraMovePID = ParentProcessID
            os.kill(ParentProcessID, signal.SIGUSR2)   

def handler(signum, frame):
    print('OakD-Camera ->handler() for ALRMi called', signum)
    raise OSError("Couldn't open device!")

## Signal Handler for SIGUSR1 , Interface from YanthraMove
def handlerSIGUSR2(signum, frame):
    print('OakD-Camera ->handlerSIGUSR2() called with signal', signum)
    #print("Waiting for 10 seconds")
    #time.sleep(10) 

if __name__ == "__main__":
    # Set the signal handler and a 5-second alarm
    signal.signal(signal.SIGALRM, handler)
    #signal.alarm(5)
    signal.signal(signal.SIGUSR1, handlerSIGUSR2) 
    main() 
  


        rgb_o3d = o3d.geometry.Image(rgb)
        depth_o3d = o3d.geometry.Image(depth_map)

        if len(rgb.shape) == 3:
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_o3d, depth_o3d, convert_rgb_to_intensity=False)
        else:
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_o3d, depth_o3d)

        if self.pcl is None:
            self.pcl = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, self.pinhole_camera_intrinsic)
        else:
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, self.pinhole_camera_intrinsic)
