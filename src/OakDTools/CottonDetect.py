#!/usr/bin/env python3

#!/usr/bin/env python3

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
IMAGELOGGING = True
#ARUCOLOG  = True
ARUCOLOG  =  False
COLOR = True
OUTPUTFILEPATH =  "/home/ubuntu/pragati/inputs/"
IMG100FILEPATH =  "/home/ubuntu/pragati/inputs/img100.jpg"
COTTONDETAILSTXTFILEPATH = "/home/ubuntu/pragati/outputs/cotton_details.txt"
DETECTIONOUTPFILEPATH = "/home/ubuntu/pragati/outputs/DetectionOutput.jpg"
#IMG100FILEPATH =  "./img100.jpg"
#COTTONDETAILSTXTFILEPATH = "./cotton_details.txt"
#DETECTIONOUTPFILEPATH = "./DetectionOutput.jpg"
logfile = open("/tmp/CottonDetectCommunicationLog.txt","w")
#sys.stdout = logfile
#sys.stderr = logfile
print ("Logging CottonDetect.py logs to /tmp/CottonDetectCommunicationLog.txt\n")

Initialisation_time_start = time.monotonic()
# Get argument first
nnBlobPath = str((Path(__file__).parent / Path('best_openvino_2022.1_6shave.blob')).resolve().absolute())
#nnBlobPath = str((Path(__file__).parent / Path('best_openvino_2022.1_6shave.blob')).resolve().absolute())
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
#We assumed Right depth camera as origin So Removing Aligning
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
manip=pipeline.create(dai.node.ImageManip)
manipOut=pipeline.createXLinkOut()
manipOut.setStreamName("manip")

stereo.initialConfig.setConfidenceThreshold(255) # 27JUL2023 Moved this before calling StereoDepth Preset Mode
stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
#stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

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
#camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

######### Setting Camera Parameters
#camRgb.setInterleaved(False)
#camRgb.setIspScale(1, 3)
#camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
# Used only for Autofocus camera. We plan to use the fixed Focus cameras
#camRgb.initialControl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
#camRgb.initialControl.setAutoExposureEnable()
#camRgb.initialControl.setutoExposureEnable()
#camRgb.initialControl.setManualExposure(200,350)
#camRgb.initialControl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)
#camRgb.initialControl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
#camRgb.initialControl.setManualFocus(130)
#stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
#camRgb.isp.link(xout_colorize.input)
#camRgb.preview.link(xout_colorize.input)

#######  END OF Setting Camera Parameters



monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# setting node configs
lrcheck = True
subpixel = False
ExtendedDisparity = True
#stereo.initialConfig.setConfidenceThreshold(255)
stereo.setLeftRightCheck(lrcheck)
stereo.setSubpixel(subpixel)
## BUG: System hangs If EXTENDEDDISPARITY is True While LRCHECK & Subpixel is True
stereo.setExtendedDisparity(ExtendedDisparity)

spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

"""
## Filter Settings  Added on 27JULY2023
config = stereo.initialConfig.get()
config.postProcessing.speckleFilter.enable = True
config.postProcessing.speckleFilter.speckleRange = 50
config.postProcessing.temporalFilter.enable = True
config.postProcessing.temporalFilter.persistencyMode = dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4  
config.postProcessing.spatialFilter.enable = True
config.postProcessing.spatialFilter.holeFillingRadius = 2 
config.postProcessing.spatialFilter.numIterations = 5 
#config.postProcessing.thresholdFilter.minRange = 300
#config.postProcessing.thresholdFilter.maxRange = 1000
config.postProcessing.decimationFilter.decimationFactor = 1
stereo.initialConfig.set(config)
# End of Filter Settings 
"""
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

#Multipliation Factor for Camera Reference Frame Change
Y_Multiplication_Factor=-1
## This is required for PCD function generation
#import projector_device
from projector_device import PointCloudVisualizer

class HostSync:
    def __init__(self):
        self.arrays = {}
    def add_msg(self, name, msg):
        if not name in self.arrays:
            self.arrays[name] = []
        # Add msg to array
        self.arrays[name].append({'msg': msg, 'seq': msg.getSequenceNum()})

        synced = {}
        for name, arr in self.arrays.items():
            for i, obj in enumerate(arr):
                if msg.getSequenceNum() == obj['seq']:
                    synced[name] = obj['msg']
                    break
        # If there are 3 (all) synced msgs, remove all old msgs
        # and return synced msgs
        if len(synced) == 2: # color, depth, nn
            # Remove old msgs
            for name, arr in self.arrays.items():
                for i, obj in enumerate(arr):
                    if obj['seq'] < msg.getSequenceNum():
                        arr.remove(obj)
                    else: break
            return synced
        return False


def GeneratePCD():
   
    qs = []
    qs.append(device.getOutputQueue("depth", 1))
    qs.append(device.getOutputQueue("rgb",1))
    #time.sleep(5)
    calibData = device.readCalibration()
    if COLOR:
        w, h = camRgb.getIspSize()
        intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RGB, dai.Size2f(w, h))
    else:
        w, h = monoRight.getResolutionSize()
        intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT, dai.Size2f(w, h))
    pcl_converter = PointCloudVisualizer(intrinsics, w, h)
    sync = HostSync()
    print(f"Getting Frames from SynchFrames",)
    for q in qs:
        new_msg = q.tryGet()
        if (new_msg is None): print(f"** Failed to Get Frames from SynchFrames")
        if new_msg is not None:
            msgs = sync.add_msg(q.getName(), new_msg)
            #print(f"**Gettinig SynchFrame s: {FrameNoFromQueue}")
            if msgs:
                depth = msgs["depth"].getFrame()
                color = msgs["colorize"].getCvFrame()
                #print( f"Got from Synch Msg FrameNo : {FrameNoFromQueue}")
                #cv2.imshow("color", color)
                cv2.imwrite("arucoImg.jpg", color)
                rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
                pcl_converter.rgbd_to_projection(depth, rgb,True)
                pcl_converter.visualize_pcd()
                pcl_converter.write_pointcloud(OUTPUTFILEPATH + "points100.pcd")
### End of GeneratePCD: 


def WaitOnSignal():
    global DetectionOutputRequired 
    print(f"CottonDetect WaitOnSignal : Waiting for SIGUSR1")
    sig = signal.sigwait([signal.SIGTERM, signal.SIGUSR1, signal.SIGHUP])
    print(f"CottonDetect WaitOnSignal : Received a Signal")
    sys.stdout.flush() 
    logfile.flush()
    if (sig == signal.SIGUSR1):
            #PrintCottonDetailsTxtFile()
            print("CottonDetect : Got SIGUSR1" )
            #DetectCotton()
            DetectionOutputRequired = True
            #continue
    if (sig == signal.SIGHUP):
            #self.load_configuration()
            print("CottonDetect : Got SIGHUP" )
            #continue
    
    if (sig == signal.SIGTERM):
            #logging.info('Got SIGTERM, exiting
            print("CottonDetect : Got SIGTERM" )
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

def PrintFlush( CommandString ):
    print( CommandString )
    sys.stdout.flush()
    logfile.flush()

def CleanUpFile(Path):
    PrintFlush("Removing Files from directory" + Path)
    if os.path.isfile(Path): os.remove(Path)
    else: print (Path + " Does not exist")

def SendReadyToParent(ParentProcessID):
     #PrintFlush("*** Sending Ready to Host ** dummied\n")
     os.kill(ParentProcessID, signal.SIGUSR2)
   
#signal.signal(signal.SIGUSR1,Detect_cotton)

# Connect to device and start pipeline
def main():
    global device
    global count
    global inDet
    global DetectionOutputRequired 
    count = 0 
    ParentProcessID = os.getppid()
    ## TODO pass down parent PID or find a reliable
    #Find Parent Process ID or find it by name  
    print(f"ParentProcessID : {ParentProcessID}")    
    CleanUpFile(IMG100FILEPATH)
    CleanUpFile(COTTONDETAILSTXTFILEPATH)
    CleanUpFile(DETECTIONOUTPFILEPATH)

#Forcing Usb Connection to USB2MODE
    with dai.Device(pipeline ,usb2Mode=True) as device:

#By Deafault the connection is in USB3.0
#  with dai.Device(pipeline) as device:
        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        xoutBoundingBoxDepthMappingQueue = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
        depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        qman = device.getOutputQueue(name="manip", maxSize=4, blocking=False)
        Initialisation_time_finish = time.monotonic()
        Initialisation_time= Initialisation_time_finish - Initialisation_time_start
        
        #PCLqueue = device.getOutputQueue("pcl", maxSize=8, blocking=False)
        print(f"Initialisation_time:{Initialisation_time}\n")
    
        startTime = time.monotonic()
        counter = 0
        ImageCount = 0
        fps = 0
        color = (255, 255, 255)
        # System Ready for Detection after initialisation
        # Send Signal to parent the ready Status & Then wait for Command from parent
        #os.kill(ParentProcessID, signal.SIGUSR2)
        SendReadyToParent(ParentProcessID)
        #PrintFlush("*** Sending Ready to Host ** dummied\n"
        print("Sent Signal SIGUSR2 (CameraReady) to Parent ")
        sys.stdout.flush() 
        logfile.flush()
        while True:
            PrintFlush (f"Stepping to WaitOnSignal\n")
            WaitOnSignal()
            if (DetectionOutputRequired == True):
               PrintFlush("DetectionOutputRequired is True\n")
               PrintFlush("Will generate cotton_detect.txt file\n")
               sys.stdout.flush() 
               logfile.flush()
               DetectionOutputRequired = False 
               try:
                     PrintFlush( "Waiting for preview  Queue ")
                     inPreview = previewQueue.get()
                     PrintFlush( "Waiting for Detection Queue ")
                     inDet = detectionNNQueue.get()
                     PrintFlush( "Waiting for  Depth Queue ")
                     depth = depthQueue.get()
                     print("FRAME RECEIVED SUCCESSFULLY")
               except:
                     print("COULD NOT GET FRAME,XLINK ERROR")
                     print("Waiting For TIMEOUT ERROR")
                     exit()
                     #print("Killing all ROS NODES")
                     #os.system(' rosnode kill -a')
#TODO:We have to indicate it to yanthra
               frame = inPreview.getCvFrame()
               depthFrame = depth.getFrame()
               depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
               depthFrameColor = cv2.equalizeHist(depthFrameColor)
               depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)
               if (IMAGELOGGING == True): 
                   #cv2.imwrite("img100.jpg"  ,frame)   
                   if(  cv2.imwrite(IMG100FILEPATH  ,frame) ) : 
                       print("Not Able to write File " + IMG100FILEPATH)
                   else: print( "Wrote File " + IMG100FILEPATH)

               if (ARUCOLOG):       GeneratePCD()


               counter+=1
               current_time = time.monotonic()
               if (current_time - startTime) > 1 :
                   fps = counter / (current_time - startTime)
                   counter = 0
                   startTime = current_time
               detections = inDet.detections
               PrintFlush("Detections Done *** \n")
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
                       txt = txt + "636 0 " + str(round(float(t.spatialCoordinates.x) / 1000, 5)) + " " + str(round(float(t.spatialCoordinates.y) / 1000, 5)*Y_Multiplication_Factor) + " " + str(round(float(t.spatialCoordinates.z) / 1000, 5)) + "\n"
                   file2 = open(COTTONDETAILSTXTFILEPATH, "w+")
                   file2.write(txt)
                   file2.close()
                   print(f"Wrote to file : {COTTONDETAILSTXTFILEPATH}\n")
                   #cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
                   #cv2.imshow("depth", depthFrameColor)
                   #cv2.imshow("rgb", frame)
                   cv2.imwrite(DETECTIONOUTPFILEPATH ,frame)
                   print(f"Wrote to file : {DETECTIONOUTPFILEPATH}\n")
                   #cv2.imwrite("OakD-DetectionOutput.jpg",frame)
                   #### Endof if (DetectionOutputsRequired)
                  #if cv2.waitKey(1) == ord('q'):
                  #    break
                  #inDet = detectionNNQueue.get()
            sys.stdout.flush()
            logfile.flush()  
            #WaitOnSignal()
            print(f"CottonDetect : Sending SIGUSR2 signal to ParentProcess at {time.ctime()} \n" )
            #YanthraMovePID = ParentProcessID
            # Signal Parent that a SIGUSR1 (or anyother signal has arrived
            # Only is SIGUSR1 had arrived will the detection output wll be written 
            SendReadyToParent( ParentProcessID ) 
            #os.kill(ParentProcessID, signal.SIGUSR2)   
            sys.stdout.flush()
            logfile.flush()  

def handler(signum, frame):
    print('OakD-Camera ->handler() for ALRMi called', signum)
    raise OSError("Couldn't open device!")

## Signal Handler for SIGUSR1 , Interface from YanthraMove
def handlerSIGUSR2(signum, frame):
    print(f"OakD-Camera ->handlerSIGUSR2() called with signal :{signum}\n")
    #print("Waiting for 10 seconds")
    #time.sleep(10) 

if __name__ == "__main__":
    # Set the signal handler and a 5-second alarm
    signal.signal(signal.SIGALRM, handler)
    print("Installed Signal Handle for SIGALRM\n")
    #signal.alarm(5)
    #signal.signal(signal.SIGUSR2, handlerSIGUSR2)
    #print("Installed Signal Handler for SIGUSR1\n")
    signal.signal(signal.SIGUSR1, handlerSIGUSR2) 
    print("Installed Signal Handler for SIGUSR2\n")
    main() 

def DontKnow() :                   
                   #pcd = open3d.geometry.PointCloud.create_from_depth_image(depthFrame)
                   #pcl_data = np.array(PCDqueue.get().getFirstLayerFp16()).reshape(1, 3, resolution[1], resolution[0])
                   #pcl_data = pcl_data.reshape(3, -1).T.astype(np.float64) / 1000.0
                   pcl_converter.visualize_pcl(pcl_data, downsample=downsample_pcl)
                   open3d.io.write_point_cloud("DetectionOutput.pcd", pcl_data)
### ENDOF  def DontKnow() :
