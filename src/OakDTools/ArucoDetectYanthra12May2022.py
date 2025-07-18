#!/usr/bin/env python3

import cv2
import depthai as dai
import pickle as pkl
import numpy as np
import os
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy.spatial.transform import Rotation as R
from cv2 import aruco
from calc import HostSpatialsCalc
from utility import *
import math
import time
# Start defining a pipeline
pipeline = dai.Pipeline()

# Define a source - two mono (grayscale) cameras
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()

xoutDepth = pipeline.createXLinkOut()
xoutRight = pipeline.createXLinkOut()


xoutDepth.setStreamName("depth")
xoutRight.setStreamName('right')
# MonoCamera
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
monoRight.out.link(xoutRight.input)

outputDepth = True
outputRectified = False
lrcheck = False
subpixel = False

# StereoDepth
stereo.setOutputDepth(outputDepth)
stereo.setOutputRectified(outputRectified)
stereo.setConfidenceThreshold(255)

stereo.setLeftRightCheck(lrcheck)
stereo.setSubpixel(subpixel)

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

stereo.depth.link(xoutDepth.input)

# Adding Filters 
# Spatial Filters
StereoDepthConfig = dai.StereoDepthConfig

# Spatial Filters
StereoDepthConfig.PostProcessing.SpatialFilter.holeFillingRadius = 3
StereoDepthConfig.PostProcessing.SpatialFilter.numIterations = 1
StereoDepthConfig.PostProcessing.SpatialFilter.alpha =  0.5
StereoDepthConfig.PostProcessing.SpatialFilter.delta = 20
StereoDepthConfig.PostProcessing.SpatialFilter.enable = True
#temporal Filters
StereoDepthConfig.PostProcessing.TemporalFilter.enable = True
StereoDepthConfig.PostProcessing.TemporalFilter.alpha = 0.4
StereoDepthConfig.PostProcessing.TemporalFilter.persistencyMode =dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4 
StereoDepthConfig.PostProcessing.TemporalFilter.delta = 20 
#Speeckle Filters
StereoDepthConfig.PostProcessing.SpeckleFilter.enable = True 
StereoDepthConfig.PostProcessing.SpeckleFilter.speckleRange = 50


dai.StereoDepthConfig.PostProcessing.SpatialFilter.holeFillingRadius = 3
dai.StereoDepthConfig.PostProcessing.SpatialFilter.numIterations = 1
dai.StereoDepthConfig.PostProcessing.SpatialFilter.alpha =  0.5
dai.StereoDepthConfig.PostProcessing.SpatialFilter.delta = 20
dai.StereoDepthConfig.PostProcessing.SpatialFilter.enable = True

# Temporal Filters
dai.StereoDepthConfig.PostProcessing.TemporalFilter.enable = True
dai.StereoDepthConfig.PostProcessing.TemporalFilter.alpha = 0.4
dai.StereoDepthConfig.PostProcessing.TemporalFilter.persistencyMode =dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4 
dai.StereoDepthConfig.PostProcessing.TemporalFilter.delta = 20 
# Speckle Filters
dai.StereoDepthConfig.PostProcessing.SpeckleFilter.enable = True 
dai.StereoDepthConfig.PostProcessing.SpeckleFilter.speckleRange = 50

#dai.StereoDepthConfig.set(StereoDepthConfig)

device = dai.Device(pipeline)
device.startPipeline()

# Output queue will be used to get the depth frames from the outputs defined above
depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)

#ArUco declaration
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters_create()
text = TextHelper()
hostSpatials = HostSpatialsCalc(device)
delta=2
hostSpatials.setDeltaRoi(delta)
#TODO check whether file is open else exit
#TODO Change file absolute path to a PARAMETER
CentroidFilePath =  "/tmp/centroid.txt"
CottonDetailsTxtFilePath = "/tmp/cotton_details"
centroid_file = open(CentroidFilePath,'w')
CottonDetailsTxtFile = open(CottonDetailsTxtFilePath + time.strftime("%d%b%Y:%H:%M:%S",time.gmtime()) + ".txt",'w')

#centroid_file = open('/home/ubuntu/.ros/centroid.txt', 'w')
#CottonDetailsTxtFile = open("/home/ubuntu/pragati/outputs/cotton_details" + time.strftime("%d%b%Y:%H:%M:%S",time.gmtime()) + ".txt")
while True:
    time.sleep(5)
    inDepth = depthQueue.get().getFrame() # blocking call, will wait until a new data has arrived
    inRight = qRight.tryGet()

    if inRight is not None:
        frameRight = inRight.getCvFrame()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frameRight, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frameRight.copy(), corners, ids)
        if len(corners) > 0:
                # flatten the ArUco IDs list
                ids = ids.flatten()
                # loop over the detected ArUCo corners
                for (markerCorner, markerID) in zip(corners, ids):
                    # extract the marker corners (which are always returned in
                    # top-left, top-right, bottom-right, and bottom-left order)
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    # convert each of the (x, y)-coordinate pairs to integers
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))
                    top_left_spatial, centroid = hostSpatials.calc_spatials(inDepth, (topLeft[0],topLeft[1]))
                    top_right_spatial, centroid = hostSpatials.calc_spatials(inDepth, (topRight[0],topRight[1]))
                    bottom_right_spatial, centroid = hostSpatials.calc_spatials(inDepth, (bottomRight[0],bottomRight[1]))
                    bottom_left_spatial, centroid = hostSpatials.calc_spatials(inDepth, (bottomLeft[0],bottomLeft[1]))

                    #draw rectangles in all the corners of the detected aruco markers
                    text.rectangle(frameRight, (topLeft[0]-delta, topLeft[1]-delta), (topLeft[0]+delta, topLeft[1]+delta))
                    text.putText(frameRight, "X: " + ("{:.1f}".format(top_left_spatial['x']/1000) if not math.isnan(top_left_spatial['x']) else "--")+"Y: " + ("{:.1f}".format(top_left_spatial['y']/1000) if not math.isnan(top_left_spatial['y']) else "--")+"Z: " + ("{:.1f}".format(top_left_spatial['z']/1000) if not math.isnan(top_left_spatial['z']) else "--"), (topLeft[0] - 50, topLeft[1] - 30))
                  
                    text.rectangle(frameRight, (topRight[0]-delta, topRight[1]-delta), (topRight[0]+delta, topRight[1]+delta))
                    text.putText(frameRight, "X: " + ("{:.1f}".format(top_right_spatial['x']/1000) if not math.isnan(top_right_spatial['x']) else "--")+"Y: " + ("{:.1f}".format(top_right_spatial['y']/1000) if not math.isnan(top_right_spatial['y']) else "--")+"Z: " + ("{:.1f}".format(top_right_spatial['z']/1000) if not math.isnan(top_right_spatial['z']) else "--"), (topRight[0] + 10, topRight[1] + 10))
        
                    text.rectangle(frameRight, (bottomRight[0]-delta, bottomRight[1]-delta), (bottomRight[0]+delta, bottomRight[1]+delta))
                    text.putText(frameRight, "X: " + ("{:.1f}".format(bottom_right_spatial['x']/1000) if not math.isnan(bottom_right_spatial['x']) else "--")+"Y: " + ("{:.1f}".format(bottom_right_spatial['y']/1000) if not math.isnan(bottom_right_spatial['y']) else "--")+"Z: " + ("{:.1f}".format(bottom_right_spatial['z']/1000) if not math.isnan(bottom_right_spatial['z']) else "--"), (bottomRight[0] + 10, bottomRight[1] + 10))

                    text.rectangle(frameRight, (bottomLeft[0]-delta, bottomLeft[1]-delta), (bottomLeft[0]+delta, bottomLeft[1]+delta))
                    text.putText(frameRight, "X: " + ("{:.1f}".format(bottom_left_spatial['x']/1000) if not math.isnan(bottom_left_spatial['x']) else "--")+"Y: " + ("{:.1f}".format(bottom_left_spatial['y']/1000) if not math.isnan(bottom_left_spatial['y']) else "--")+"Z: " + ("{:.1f}".format(bottom_left_spatial['z']/1000) if not math.isnan(bottom_left_spatial['z']) else "--"), (bottomLeft[0] -50, bottomLeft[1]+30))
                cv2.imshow("aruco_corner_spatials",frameRight)
                cv2.imwrite("ArucoCornerSpatials.jpg",frameRight)
        else :
            print("ArucoDetection failed") 
            exit(1) 
        key = cv2.waitKey(1)
        if key == ord('q'):
            cv2.imshow("aruco_corner_spatials",frameRight)
            cv2.imwrite("ArucoCornerSpatials.jpg",frameRight)

    #key = cv2.waitKey(1)
    #if key == ord('q'):
    if ( True ):
        ContentTxt = "{0:.3f} ".format(top_left_spatial['x']/1000) 
        ContentTxt += "{0:.3f} ".format(top_left_spatial['y']/1000) 
        ContentTxt += "{0:.3f} ".format(top_left_spatial['z']/1000)  +"\n"
        ContentTxt += "{0:.3f} ".format(top_right_spatial['x']/1000) 
        ContentTxt += "{0:.3f} ".format(top_right_spatial['y']/1000) 
        ContentTxt += "{0:.3f} ".format(top_right_spatial['z']/1000)  +"\n"
        ContentTxt += "{0:.3f} ".format(bottom_right_spatial['x']/1000) 
        ContentTxt += "{0:.3f} ".format(bottom_right_spatial['y']/1000) 
        ContentTxt += "{0:.3f} ".format(bottom_right_spatial['z']/1000)   +"\n"
        ContentTxt += "{0:.3f} ".format(bottom_left_spatial['x']/1000) 
        ContentTxt += "{0:.3f} ".format(bottom_left_spatial['y']/1000) 
        ContentTxt += "{0:.3f} ".format(bottom_left_spatial['z']/1000)   +"\n"

        CottonDetailsTxt = "0  0" + "{0:.3f} ".format(top_left_spatial['x']/1000) 
        CottonDetailsTxt = "0  0" + "{0:.3f} ".format(top_left_spatial['y']/1000) 
        CottonDetailsTxt = "0 0" + "{0:.3f} ".format(top_left_spatial['z']/1000)  +"\n"
        CottonDetailsTxt = "0 0" + "{0:.3f} ".format(top_right_spatial['x']/1000) 
        CottonDetailsTxt = "0 0" + "{0:.3f} ".format(top_right_spatial['y']/1000) 
        CottonDetailsTxt = "0 0" + "{0:.3f} ".format(top_right_spatial['z']/1000)  +"\n"
        CottonDetailsTxt = "0 0" + "{0:.3f} ".format(bottom_right_spatial['x']/1000) 
        CottonDetailsTxt = "0 0" + "{0:.3f} ".format(bottom_right_spatial['y']/1000) 
        CottonDetailsTxt = "0 0" + "{0:.3f} ".format(bottom_right_spatial['z']/1000)   +"\n"
        CottonDetailsTxt = "0 0" + "{0:.3f} ".format(bottom_left_spatial['x']/1000) 
        CottonDetailsTxt = "0 0" + "{0:.3f} ".format(bottom_left_spatial['y']/1000) 
        CottonDetailsTxt = "0 0" + "{0:.3f} ".format(bottom_left_spatial['z']/1000)   +"\n"
        content=f"{top_left_spatial['x']/1000} {top_left_spatial['y']/1000} {top_left_spatial['z']/1000}\n{top_right_spatial['x']/1000} {top_right_spatial['y']/1000} {top_right_spatial['z']/1000}\n{bottom_right_spatial['x']/1000} {bottom_right_spatial['y']/1000} {bottom_right_spatial['z']/1000}\n{bottom_left_spatial['x']/1000} {bottom_left_spatial['y']/1000} {bottom_left_spatial['z']/1000}\n" 
        #centroid_file.write(content)
        # TODO check whether file is written
        centroid_file.write(ContentTxt)
        centroid_file.close()
        CottonDetailsTxtFile.write(CottonDetailsTxt)
        CottonDetailsTxtFile.close()
        print(" Wrote to  centroid.txt")
        print(ContentTxt)
        exit(0)
        break
                
    
    
    
