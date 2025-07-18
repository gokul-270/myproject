#!/usr/bin/env python3

import cv2
import depthai as dai
import pickle as pkl
import numpy as np
import os
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
#from scipy.spatial.transform import Rotation as R
from cv2 import aruco
from calc import HostSpatialsCalc
from utility import *
import math
import time

#######  File Inputs and File outputs for logging Aruco Detector processing
#TODO check whether file is open else exit
#TODO Change file absolute path to a PARAMETER
#CentroidFilePath =  "/tmp/centroid.txt"
#CottonDetailsTxtFilePath = "/tmp/cotton_details"
CentroidFilePath = '/home/ubuntu/.ros/centroid.txt'
CottonDetailsTxtFilePath = "/home/ubuntu/pragati/outputs/cotton_details"
ArucoDetectorInputImage = "/home/ubuntu/pragati/inputs/ArucoInputImage.jpg"
ArucoDetectorOutputImage = "/home/ubuntu/pragati/outputs/ArucoDetectorOutput.jpg"
centroid_file = open(CentroidFilePath,'w')
CottonDetailsTxtFilePath = "/home/ubuntu/pragati/outputs/cotton_details.txt"
#CottonDetailsTxtFile = open(CottonDetailsTxtFilePath + time.strftime("%d%b%Y:%H:%M:%S",time.gmtime()) + ".txt",'w')
CottonDetailsTxtFile = open(CottonDetailsTxtFilePath,'w')
######### File Inputs and File outputs for logging Aruco Detector processing

# Start defining a pipeline
pipeline = dai.Pipeline()
"""Applying all the changes to code
outDepth = True          # Disparity by default
outConfidenceMap = True  # Output disparity confidence map
#outRectified = True      # Output and display rectified streams
lrcheck = True           # Better handling for occlusions
extended = False         # Closer-in minimum depth, disparity range is doubled. Unsupported for now.
subpixel = True          # Better accuracy for longer distance, fractional disparity 

frame = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)
Median Filter Kernel 7*7 is better than Median Off and Median Filter 5*5,4*4,3*3    

disparityMultiplier = 255/ stereo.getMaxDisparity()

depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth = pipeline.create(dai.node.StereoDepth)



"""
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

outputDepth =True
outputRectified =True
#outConfidenceMap = True  # Output disparity confidence map
lrcheck = True
subpixel = True
extendedDisparity = False         # Closer-in minimum depth, disparity range is doubled. Unsupported for now.

# StereoDepth
stereo.setOutputDepth(outputDepth)
stereo.setOutputRectified(outputRectified)
stereo.setConfidenceThreshold(255)

stereo.setLeftRightCheck(lrcheck)
stereo.setSubpixel(subpixel)

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

stereo.depth.link(xoutDepth.input)
stereo.setExtendedDisparity(extendedDisparity)
# Adding Filters 
stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

 ## Filter Settings 
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
'''
# Spatial Filters
StereoDepthConfig = stereo.initialConfig.get()
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
'''

'''
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
stereo.initialConfig.set(StereoDepthConfig)
# StereoDepthConfig.set() #StereoDepthConfig)
'''

# Disparity range is used for normalization
#disparityMultiplier = 255 / stereo.initialConfig.getMaxDisparity()

device = dai.Device(pipeline, usb2Mode=True)
device.startPipeline()
#device = dai.Device()
#device.startPipeline(pipeline)

# Output queue will be used to get the depth frames from the outputs defined above
depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
if (depthQueue is None):
    print("Not getting Depth Queue  ")

qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)
if (qRight is None):
    print("Not getting Right Queue  ")

#ArUco declaration
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters_create()
text = TextHelper()
hostSpatials = HostSpatialsCalc(device)
delta=2
hostSpatials.setDeltaRoi(delta)

while True:

    inRight = None
    print (None) 
    print (inRight)
    print("waiting for 5 seconds before capturing image")
    time.sleep(5) #//TODO:Changing to 0.5 sec to test marc quickly
    inDepth = depthQueue.get().getFrame() # blocking call, will wait until a new data has arrived
    inRight = qRight.tryGet()
    print(inRight)
    if (inRight is None ): 
        print ("Error OakD not capturing Frames ")
        print ("Error : Check Camera")
        exit(1)

    '''try:
        inDepth = depthQueue.get().getFrame() # blocking call, will wait until a new data has arrived
    except:
        print("Error Getting Depth Frames")
    try:
       inRight = qRight.tryGet()
    except :
        print("Error : Not capturing Right Camera Frames")
    '''

    if inRight is not None:
        frameRight = inRight.getCvFrame()
        ## the Following for Debug purposes only 
        #cv2.imwrite(ArucoDetectorInputImage ,frameRight)
        if (cv2.imwrite("/home/ubuntu/pragati/inputs/ArucoInputImage.jpg",frameRight)) :
            print ("ArucoDetector.py : Wrote ~/pragati/inputs/ArucoInputImage.jpg")
        else :
            print ("ArucoDetector.py : Not able to write  ~/pragati/inputs/ArucoInputImage.jpg")
            #print ("ArucoDetector.py : Error Exiting Program")
            #exit(1)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(frameRight, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frameRight.copy(), corners, ids)
 
        print ("No Of Aruco Corner Found : {}".format(len(corners)))
        if (len (corners) <= 0) : 
            print ("ArucoMarkerDetect : Failed to detect corners ")
            print ("ArucoDetector.py : Error Exiting Program")
            exit(1)

        if (len(corners) > 0) :
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
                    text.putText(frameRight, "X: " + ("{:.3f}".format(top_left_spatial['x']/1000) if not math.isnan(top_left_spatial['x']) else "--")+"Y: " + ("{:.3f}".format(top_left_spatial['y']/1000) if not math.isnan(top_left_spatial['y']) else "--")+"Z: " + ("{:.3f}".format(top_left_spatial['z']/1000) if not math.isnan(top_left_spatial['z']) else "--"), (topLeft[0] - 50, topLeft[1] - 30))
                  
                    text.rectangle(frameRight, (topRight[0]-delta, topRight[1]-delta), (topRight[0]+delta, topRight[1]+delta))
                    text.putText(frameRight, "X: " + ("{:.3f}".format(top_right_spatial['x']/1000) if not math.isnan(top_right_spatial['x']) else "--")+"Y: " + ("{:.3f}".format(top_right_spatial['y']/1000) if not math.isnan(top_right_spatial['y']) else "--")+"Z: " + ("{:.3f}".format(top_right_spatial['z']/1000) if not math.isnan(top_right_spatial['z']) else "--"), (topRight[0] + 10, topRight[1] + 10))
        
                    text.rectangle(frameRight, (bottomRight[0]-delta, bottomRight[1]-delta), (bottomRight[0]+delta, bottomRight[1]+delta))
                    text.putText(frameRight, "X: " + ("{:.3f}".format(bottom_right_spatial['x']/1000) if not math.isnan(bottom_right_spatial['x']) else "--")+"Y: " + ("{:.3f}".format(bottom_right_spatial['y']/1000) if not math.isnan(bottom_right_spatial['y']) else "--")+"Z: " + ("{:.3f}".format(bottom_right_spatial['z']/1000) if not math.isnan(bottom_right_spatial['z']) else "--"), (bottomRight[0] + 10, bottomRight[1] + 10))

                    text.rectangle(frameRight, (bottomLeft[0]-delta, bottomLeft[1]-delta), (bottomLeft[0]+delta, bottomLeft[1]+delta))
                    text.putText(frameRight, "X: " + ("{:.3f}".format(bottom_left_spatial['x']/1000) if not math.isnan(bottom_left_spatial['x']) else "--")+"Y: " + ("{:.3f}".format(bottom_left_spatial['y']/1000) if not math.isnan(bottom_left_spatial['y']) else "--")+"Z: " + ("{:.3f}".format(bottom_left_spatial['z']/1000) if not math.isnan(bottom_left_spatial['z']) else "--"), (bottomLeft[0] -50, bottomLeft[1]+30))
                #cv2.imshow("aruco_corner_spatials",frameRight)
                #cv2.imwrite("ArucoCornerSpatials.jpg",frameRight)
                cv2.imwrite("ArucoDetectorOutputImage.jpg",frameRight)
                # The following line is not working
                #cv2.imwrite(ArucoDetectorOutputImage,frameRight)
                if(cv2.imwrite("/home/ubuntu/pragati/outputs/ArucoDetectorOutput.jpg",frameRight)): print ("ArucoDetector.py : Writing to  /home/ubuntu/pragati/outputs/ArucoDetectorOutput.jpg")
        else:
            print("ArucoDetection failed") 
            exit(1)

            
        ''' Used for Debugging Only
        key = cv2.waitKey(1)
        if key == ord('q'):
            cv2.imshow("aruco_corner_spatials",frameRight)
            cv2.imwrite("ArucoCornerSpatials.jpg",frameRight)
        '''


    y_muliplication_factor= -1 #Hack for Coordinate Systm Unsync between OAK and YanthraTODO
    #key = cv2.waitKey(1)
    #if key == ord('q'):
    if ( True ):
        ContentTxt = "{0:.3f} ".format(top_left_spatial['x']/1000) 
        ContentTxt += "{0:.3f} ".format(top_left_spatial['y']*y_muliplication_factor/1000) 
        ContentTxt += "{0:.3f} ".format(top_left_spatial['z']/1000)  +"\n"
        ContentTxt += "{0:.3f} ".format(top_right_spatial['x']/1000) 
        ContentTxt += "{0:.3f} ".format(top_right_spatial['y']*y_muliplication_factor/1000) 
        ContentTxt += "{0:.3f} ".format(top_right_spatial['z']/1000)  +"\n"
        ContentTxt += "{0:.3f} ".format(bottom_right_spatial['x']/1000) 
        ContentTxt += "{0:.3f} ".format(bottom_right_spatial['y']*y_muliplication_factor/1000) 
        ContentTxt += "{0:.3f} ".format(bottom_right_spatial['z']/1000)   +"\n"
        ContentTxt += "{0:.3f} ".format(bottom_left_spatial['x']/1000) 
        ContentTxt += "{0:.3f} ".format(bottom_left_spatial['y']*y_muliplication_factor/1000)
        ContentTxt += "{0:.3f} ".format(bottom_left_spatial['z']/1000)   +"\n"

        CottonDetailsTxt = "0 0 " + "{0:.3f} ".format(top_left_spatial['x']/1000) 
        CottonDetailsTxt +=  "{0:.3f} ".format(top_left_spatial['y']*y_muliplication_factor/1000) 
        CottonDetailsTxt +=  "{0:.3f} ".format(top_left_spatial['z']/1000)  +"\n"
        CottonDetailsTxt += "0 0 " + "{0:.3f} ".format(top_right_spatial['x']/1000) 
        CottonDetailsTxt +=  "{0:.3f} ".format(top_right_spatial['y']*y_muliplication_factor/1000)
        CottonDetailsTxt +=  "{0:.3f} ".format(top_right_spatial['z']/1000)  +"\n"
        CottonDetailsTxt += "0 0 " + "{0:.3f} ".format(bottom_right_spatial['x']/1000) 
        CottonDetailsTxt +=  "{0:.3f} ".format(bottom_right_spatial['y']*y_muliplication_factor/1000) 
        CottonDetailsTxt +=  "{0:.3f} ".format(bottom_right_spatial['z']/1000)   +"\n"
        CottonDetailsTxt += "0 0 " + "{0:.3f} ".format(bottom_left_spatial['x']/1000) 
        CottonDetailsTxt +=  "{0:.3f} ".format(bottom_left_spatial['y']*y_muliplication_factor/1000)
        CottonDetailsTxt +=  "{0:.3f} ".format(bottom_left_spatial['z']/1000)   +"\n"
        #content=f"{top_left_spatial['x']/1000} {top_left_spatial['y']/1000} {top_left_spatial['z']/1000}\n{top_right_spatial['x']/1000} {top_right_spatial['y']/1000} {top_right_spatial['z']/1000}\n{bottom_right_spatial['x']/1000} {bottom_right_spatial['y']/1000} {bottom_right_spatial['z']/1000}\n{bottom_left_spatial['x']/1000} {bottom_left_spatial['y']/1000} {bottom_left_spatial['z']/1000}\n" 
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
                
    
    
    
