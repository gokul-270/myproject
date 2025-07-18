#!/usr/bin/env python3
# History 
# 10 Feb 2023 Changed input queue size to match the node and the corresponding output node
#      Changed queue blocking from the default True to False. THis will result in lose of frames but will not stop the pipeline
#  1. Changed the input Queuesize of Image Manipulation node to 1
#          manip.inputConfig.setQueueSize(1)
#  2. Set the spatialDetectionNetwork.input.setQueueSize(4)
#  3. Set the spatialDetectionNetwork.inputDepth.setBlocking(False)
#  4. Set the spatialDetectionNetwork.inputDepth.setQueueSize(4)
#  5  xoutRgb.input.setQueueSize(1)
#  6. xoutNN = pipeline.createXLinkOut()
#  7. xoutNN.input.setBlocking(False)
#  8  xoutNN.input.setQueueSize(1)
##!/usr/bin/env python3
# Removing All logging Aspect of the detection 
# Removing all xlink nodes which are not used for the actual processing

from operator import truediv
from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import os
import signal
#import open3d
'''
Spatial Tiny-yolo 
  Performs inference on RGB camera and retrieves spatial location coordinates: x,y,z relative to the center of depth map.
  Can be used for tiny-yolo-v3 or tiny-yolo-v4 networks
'''
global device
global inDet
global count
global DetectionOutputRequired

DetectionOutputRequired = True
#IMAGELOGGING = "false" 
IMAGELOGGING = False
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
#logfile = open("/tmp/CottonDetectCommunicationLog.txt","w")
#sys.stdout = logfile
#sys.stderr = logfile
#print ("Logging CottonDetect.py logs to /tmp/CottonDetectCommunicationLog.txt\n")
CaptureFrameRate = 10
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
#Set the node to run on the LEON_CSS 
#spatialDetectionNetwork.setProcessor(dai.ProcessorType.LEON_CSS)
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
manip=pipeline.create(dai.node.ImageManip)
print("Manip Size of Input Queue before setting :{}".format(manip.inputConfig.getQueueSize()) )
print("Manip Size of InputImage Queue before setting :{}".format(manip.inputImage.getQueueSize()) )
print("Manip Blocking input State Before setting :{}".format(manip.inputConfig.getBlocking()) )
print("Manip Blocking inputImage State Before setting :{}".format(manip.inputImage.getBlocking()) )
manip.inputConfig.setQueueSize(1)
manip.inputImage.setQueueSize(1)
manip.inputImage.setBlocking(False)
manip.inputConfig.setBlocking(False)
print("Manip Size of Input Queue AFTER setting :{}".format(manip.inputConfig.getQueueSize()) )
print("Manip Size of InputImage Queue AFTER setting :{}".format(manip.inputImage.getQueueSize()) )
print("Manip Blocking inputImage State AFTER setting :{}".format(manip.inputImage.getBlocking()) )
print("Manip Blocking State AFTER setting :{}".format(manip.inputConfig.getBlocking()) )

#Set the node to run on the LEON_CSS 
#manip.setProcessor(dai.ProcessorType.LEON_CSS)
#manipOut=pipeline.createXLinkOut()
#manipOut.setStreamName("manip")

stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
#stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

xoutRgb = pipeline.createXLinkOut()                                                     
xoutRgb.input.setBlocking(False)
xoutRgb.input.setQueueSize(1)
xoutNN = pipeline.createXLinkOut()
xoutNN.input.setBlocking(False)
xoutNN.input.setQueueSize(1)

#xoutBoundingBoxDepthMapping = pipeline.createXLinkOut()
#xoutDepth = pipeline.createXLinkOut()

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
#xoutNN.setInput
#xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
#xoutDepth.setStreamName("depth")

# Properties
manip.initialConfig.setResizeThumbnail(416,416)

camRgb.setPreviewSize(1920,1080)
#camRgb.setPreviewSize(416, 416)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
#camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(CaptureFrameRate )
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
camRgb.setFps(CaptureFrameRate )
monoRight.setFps(CaptureFrameRate )
monoLeft.setFps(CaptureFrameRate )
print(" FPS of RGB: ", camRgb.getFps())
print(" FPS of monoLeft: ", monoLeft.getFps())
print(" FPS of monoRight: ",monoRight.getFps())
print(" Frames in Preview : ", camRgb.getPreviewNumFramesPool())
print(" Preview Size : ", camRgb.getPreviewSize())
print(" Preview Resolution Width : ", camRgb.getPreviewWidth())
print(" Preview Resolution Height: ", camRgb.getPreviewHeight())
#print(" YoloSpatialDetectionNetwork sizeof Pool Frames: ", spatialDetectionNetwork.getNumPoolFrames())
# setting node configs
lrcheck = True
subpixel = False
ExtendedDisparity = True
stereo.initialConfig.setConfidenceThreshold(255)
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

spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.input.setQueueSize(4)
spatialDetectionNetwork.inputDepth.setBlocking(False)
spatialDetectionNetwork.inputDepth.setQueueSize(4)

# Yolo specific parameters
spatialDetectionNetwork.setNumClasses(1)
spatialDetectionNetwork.setCoordinateSize(4)
spatialDetectionNetwork.setAnchors(np.array([10,13, 16,30, 33,23, 30,61, 62,45, 59,119, 116,90, 156,198, 373,326]))
spatialDetectionNetwork.setAnchorMasks({"side52": np.array([0,1,2]), "side26": np.array([3,4,5]), "side13": np.array([6,7,8]) })
spatialDetectionNetwork.setIouThreshold(0.5)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

#camRgb.preview.link(spatialDetectionNetwork.input)
camRgb.preview.link(manip.inputImage)
manip.out.link(spatialDetectionNetwork.input)
x,y = spatialDetectionNetwork.getInputs()
y.setQueueSize(1)
x.setQueueSize(1)
print( "SpatialDetectionNetwork input : {} size {}".format(y.name, y.getQueueSize()))
print( "SpatialDetectionNetwork input : {} size {}".format(x.name, x.getQueueSize())) 
if syncNN:
    spatialDetectionNetwork.passthrough.link(xoutRgb.input)
else:
    camRgb.preview.link(xoutRgb.input)
#manip.out.link(manipOut.input)
spatialDetectionNetwork.out.link(xoutNN.input)
#spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
#spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

#Multipliation Factor for Camera Reference Frame Change
Y_Multiplication_Factor=-1
## This is required for PCD function generation
#import projector_device
#from projector_device import PointCloudVisualizer

# Added on 01FEB2023
# Create a systemLogger to check device parameters such as temperature
sysLog = pipeline.create(dai.node.SystemLogger)
linkOut = pipeline.create(dai.node.XLinkOut)
linkOut.setStreamName("sysinfo")
sysLog.setRate(1)  # 1 Hz
sysLog.out.link(linkOut.input)

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
def printSystemInformation(info):
    m = 1024 * 1024 # MiB
    print(f"Ddr used / total - {info.ddrMemoryUsage.used / m:.2f} / {info.ddrMemoryUsage.total / m:.2f} MiB")
    print(f"Cmx used / total - {info.cmxMemoryUsage.used / m:.2f} / {info.cmxMemoryUsage.total / m:.2f} MiB")
    print(f"LeonCss heap used / total - {info.leonCssMemoryUsage.used / m:.2f} / {info.leonCssMemoryUsage.total / m:.2f} MiB")
    print(f"LeonMss heap used / total - {info.leonMssMemoryUsage.used / m:.2f} / {info.leonMssMemoryUsage.total / m:.2f} MiB")
    t = info.chipTemperature
    print(f"Chip temperature - average: {t.average:.2f}, css: {t.css:.2f}, mss: {t.mss:.2f}, upa: {t.upa:.2f}, dss: {t.dss:.2f}")
    print(f"Cpu usage - Leon CSS: {info.leonCssCpuUsage.average * 100:.2f}%, Leon MSS: {info.leonMssCpuUsage.average * 100:.2f} %")
    print("----------------------------------------")

def GetSystemInfo(qSysInfo):
        PrintFlush( "Waiting for SysInfo  Queue ")
        try :
           sysInfo = qSysInfo.get()  # Blocking call, will wait until a new data has arrived
           printSystemInformation(sysInfo)
        except : 
           PrintFlush(" Error Getting system Info")
           exit()
# End of           def GetSystemInfo():
'''
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
                #cv2.imwrite("arucoImg.jpg", color)
                rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
                pcl_converter.rgbd_to_projection(depth, rgb,True)
                pcl_converter.visualize_pcd()
                pcl_converter.write_pointcloud(OUTPUTFILEPATH + "points100.pcd")
### End of GeneratePCD: 
'''

def WaitOnSignal():
    global DetectionOutputRequired 
    print(f"CottonDetect WaitOnSignal : Waiting for SIGUSR1")
    sig = signal.sigwait([signal.SIGTERM, signal.SIGUSR1, signal.SIGHUP])
    print(f"CottonDetect WaitOnSignal : Received a Signal")
    #sys.stdout.flush() 
    #logfile.flush()
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
    #sys.stdout.flush()
    #logfile.flush()

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
    #CleanUpFile(IMG100FILEPATH)
    #CleanUpFile(DETECTIONOUTPFILEPATH)
    CleanUpFile(COTTONDETAILSTXTFILEPATH)

#Forcing Usb Connection to USB2MODE
    with dai.Device(pipeline) as device:
    #with dai.Device(pipeline) as device:

#By Deafault the connection is in USB3.0
#  with dai.Device(pipeline) as device:
        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        #detectionNNQueue = device.getOutputQueue(name="detections", maxSize=1, blocking=False)
        if (IMAGELOGGING == True): 
               previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
               xoutBoundingBoxDepthMappingQueue = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
               depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        ## 27 JAN 2023 , Manohar Sambandam Commenting out the following line. Not used any where
        #qman = device.getOutputQueue(name="manip", maxSize=4, blocking=False)
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
        try:  
           qSysInfo = device.getOutputQueue(name="sysinfo", maxSize=4, blocking=False)
        except :
           PrintFlush('Error Getting device.getOutputQueue(name="sysinfo", maxSize=4, blocking=False')
        #GetSystemInfo(qSysInfo)

        while True:
            PrintFlush (f"Stepping to WaitOnSignal\n")
            WaitOnSignal()
            if (DetectionOutputRequired == True):
               PrintFlush("DetectionOutputRequired is True\n")
               #GetSystemInfo(qSysInfo)
               detectionNNQueue = device.getOutputQueue(name="detections", maxSize=1, blocking=False)
               PrintFlush( "Waiting for Detection Queue ")
               inDet = detectionNNQueue.get()
               PrintFlush("Will generate cotton_detect.txt file\n")
               DetectionOutputRequired = False 

               '''
               CleanUpFile(IMG100FILEPATH)
               CleanUpFile(DETECTIONOUTPFILEPATH)
               CleanUpFile(COTTONDETAILSTXTFILEPATH)
               '''

               if (IMAGELOGGING == True): 
                   PrintFlush( "Waiting for preview  Queue ")
                   inPreview = previewQueue.get()
                   PrintFlush( "Waiting for  Depth Queue ")
                   depth = depthQueue.get()
                   frame = inPreview.getCvFrame()
                   depthFrame = depth.getFrame()
                   depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
                   depthFrameColor = cv2.equalizeHist(depthFrameColor)
                   depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)
               if (IMAGELOGGING == True): 
                  print("Generating img100 \n")
                  cv2.imwrite("img100.jpg"  ,frame)   
                  if(  cv2.imwrite(IMG100FILEPATH  ,frame) ) : 
                       print("Not Able to write File " + IMG100FILEPATH)
                  else: print( "Wrote File " + IMG100FILEPATH)

               #if (ARUCOLOG):       GeneratePCD()

               counter+=1
               current_time = time.monotonic()
               if (current_time - startTime) > 1 :
                   fps = counter / (current_time - startTime)
                   counter = 0
                   startTime = current_time
               detections = inDet.detections
               PrintFlush("Detections Done *** \n")
               if len(detections) == 0:
                   # Commenting out this part of the logic for Now
                   if (IMAGELOGGING == True): 
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
                   # End OF if (IMAGELOGGING == True): 
                   txt=""
                   for detection in detections:
                       t=detection
                       txt = txt + "636 0 " + str(round(float(t.spatialCoordinates.x) / 1000, 5)) + " " + str(round(float(t.spatialCoordinates.y) / 1000, 5)*Y_Multiplication_Factor) + " " + str(round(float(t.spatialCoordinates.z) / 1000, 5)) + "\n"
                       if (IMAGELOGGING == True): 
                           # If the frame is available, draw bounding boxes on it and show the frame
                           height = frame.shape[0]
                           width  = frame.shape[1]
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
                   file2 = open(COTTONDETAILSTXTFILEPATH, "w+")
                   file2.write(txt)
                   file2.close()
                   print(f"Wrote to file : {COTTONDETAILSTXTFILEPATH}\n")
                   #cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
                   #cv2.imshow("depth", depthFrameColor)
                   #cv2.imshow("rgb", frame)
                   if (IMAGELOGGING == True): 
                       cv2.imwrite(DETECTIONOUTPFILEPATH ,frame)
                       print(f"Wrote to file : {DETECTIONOUTPFILEPATH}\n")
                   #### Endof if (DetectionOutputsRequired)
                  #if cv2.waitKey(1) == ord('q'):
                  #    break
                  #inDet = detectionNNQueue.get()
            #WaitOnSignal()
            print(f"CottonDetect : Sending SIGUSR2 signal to ParentProcess at {time.ctime()} \n" )
            #YanthraMovePID = ParentProcessID
            # Signal Parent that a SIGUSR1 (or anyother signal has arrived
            # Only is SIGUSR1 had arrived will the detection output wll be written 
            SendReadyToParent( ParentProcessID ) 
            #os.kill(ParentProcessID, signal.SIGUSR2)   

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
    CleanUpFile(IMG100FILEPATH)
    CleanUpFile(DETECTIONOUTPFILEPATH)
    CleanUpFile(COTTONDETAILSTXTFILEPATH)
    main() 
"""
def DontKnow() :                   
                   #pcd = open3d.geometry.PointCloud.create_from_depth_image(depthFrame)
                   #pcl_data = np.array(PCDqueue.get().getFirstLayerFp16()).reshape(1, 3, resolution[1], resolution[0])
                   #pcl_data = pcl_data.reshape(3, -1).T.astype(np.float64) / 1000.0
                   pcl_converter.visualize_pcl(pcl_data, downsample=downsample_pcl)
                   #open3d.io.write_point_cloud("DetectionOutput.pcd", pcl_data)
### ENDOF  def DontKnow() :                   
"""
