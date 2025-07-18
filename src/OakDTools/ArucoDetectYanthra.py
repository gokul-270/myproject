#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import os
from cv2 import aruco
from calc import HostSpatialsCalc
from utility import TextHelper
import math
import time

# File Inputs and File outputs for logging Aruco Detector processing
CentroidFilePath = '/home/ubuntu/.ros/centroid.txt'
CottonDetailsTxtFilePath = "/home/ubuntu/pragati/outputs/cotton_details.txt"
ArucoDetectorInputImage = "/home/ubuntu/pragati/inputs/ArucoInputImage.jpg"
ArucoDetectorOutputImage = "/home/ubuntu/pragati/outputs/ArucoDetectorOutput.jpg"
centroid_file = open(CentroidFilePath, 'w')
CottonDetailsTxtFile = open(CottonDetailsTxtFilePath, 'w')

# Start defining a pipeline
pipeline = dai.Pipeline()
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
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
outputRectified = True
lrcheck = True
subpixel = True
extendedDisparity = False

# StereoDepth
stereo.initialConfig.setConfidenceThreshold(255)
stereo.setLeftRightCheck(lrcheck)
stereo.setSubpixel(subpixel)
stereo.setExtendedDisparity(extendedDisparity)
stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

# Filter Settings
config = stereo.initialConfig.get()
config.postProcessing.speckleFilter.enable = True
config.postProcessing.speckleFilter.speckleRange = 50
config.postProcessing.temporalFilter.enable = True
config.postProcessing.temporalFilter.persistencyMode = dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4
config.postProcessing.spatialFilter.enable = True
config.postProcessing.spatialFilter.holeFillingRadius = 2
config.postProcessing.spatialFilter.numIterations = 5
config.postProcessing.decimationFilter.decimationFactor = 1
stereo.initialConfig.set(config)

# Link stereo depth outputs
stereo.depth.link(xoutDepth.input)
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

device = dai.Device(pipeline, usb2Mode=True)
device.startPipeline()

# Output queue will be used to get the depth frames from the outputs defined above
depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)

# ArUco declaration
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()
text = TextHelper()
hostSpatials = HostSpatialsCalc(device)
delta = 2
hostSpatials.setDeltaRoi(delta)

while True:
    print("waiting for 5 seconds before capturing image")
    time.sleep(5)
    inDepth = depthQueue.get().getFrame()  # blocking call, will wait until a new data has arrived
    inRight = qRight.tryGet()

    if inRight is None:
        print("Error OakD not capturing Frames")
        print("Error: Check Camera")
        exit(1)

    if inRight is not None:
        frameRight = inRight.getCvFrame()
        if cv2.imwrite(ArucoDetectorInputImage, frameRight):
            print("ArucoDetector.py: Wrote ~/pragati/inputs/ArucoInputImage.jpg")
        else:
            print("ArucoDetector.py: Not able to write ~/pragati/inputs/ArucoInputImage.jpg")

        corners, ids, rejectedImgPoints = aruco.detectMarkers(frameRight, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frameRight.copy(), corners, ids)

        print("No Of Aruco Corner Found: {}".format(len(corners)))
        if len(corners) <= 0:
            print("ArucoMarkerDetect: Failed to detect corners")
            print("ArucoDetector.py: Error Exiting Program")
            exit(1)

        ContentTxt = ""
        CottonDetailsTxt = ""

        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                top_left_spatial, centroid = hostSpatials.calc_spatials(inDepth, (topLeft[0], topLeft[1]))
                top_right_spatial, centroid = hostSpatials.calc_spatials(inDepth, (topRight[0], topRight[1]))
                bottom_right_spatial, centroid = hostSpatials.calc_spatials(inDepth, (bottomRight[0], bottomRight[1]))
                bottom_left_spatial, centroid = hostSpatials.calc_spatials(inDepth, (bottomLeft[0], bottomLeft[1]))

                text.rectangle(frameRight, (topLeft[0] - delta, topLeft[1] - delta), (topLeft[0] + delta, topLeft[1] + delta))
                text.putText(frameRight, "X: " + ("{:.3f}".format(top_left_spatial['x'] / 1000) if not math.isnan(top_left_spatial['x']) else "--") + "Y: " + ("{:.3f}".format(top_left_spatial['y'] / 1000) if not math.isnan(top_left_spatial['y']) else "--") + "Z: " + ("{:.3f}".format(top_left_spatial['z'] / 1000) if not math.isnan(top_left_spatial['z']) else "--"), (topLeft[0] - 50, topLeft[1] - 30))

                text.rectangle(frameRight, (topRight[0] - delta, topRight[1] - delta), (topRight[0] + delta, topRight[1] + delta))
                text.putText(frameRight, "X: " + ("{:.3f}".format(top_right_spatial['x'] / 1000) if not math.isnan(top_right_spatial['x']) else "--") + "Y: " + ("{:.3f}".format(top_right_spatial['y'] / 1000) if not math.isnan(top_right_spatial['y']) else "--") + "Z: " + ("{:.3f}".format(top_right_spatial['z'] / 1000) if not math.isnan(top_right_spatial['z']) else "--"), (topRight[0] + 10, topRight[1] + 10))

                text.rectangle(frameRight, (bottomRight[0] - delta, bottomRight[1] - delta), (bottomRight[0] + delta, bottomRight[1] + delta))
                text.putText(frameRight, "X: " + ("{:.3f}".format(bottom_right_spatial['x'] / 1000) if not math.isnan(bottom_right_spatial['x']) else "--") + "Y: " + ("{:.3f}".format(bottom_right_spatial['y'] / 1000) if not math.isnan(bottom_right_spatial['y']) else "--") + "Z: " + ("{:.3f}".format(bottom_right_spatial['z'] / 1000) if not math.isnan(bottom_right_spatial['z']) else "--"), (bottomRight[0] + 10, bottomRight[1] + 10))

                text.rectangle(frameRight, (bottomLeft[0] - delta, bottomLeft[1] - delta), (bottomLeft[0] + delta, bottomLeft[1] + delta))
                text.putText(frameRight, "X: " + ("{:.3f}".format(bottom_left_spatial['x'] / 1000) if not math.isnan(bottom_left_spatial['x']) else "--") + "Y: " + ("{:.3f}".format(bottom_left_spatial['y'] / 1000) if not math.isnan(bottom_left_spatial['y']) else "--") + "Z: " + ("{:.3f}".format(bottom_left_spatial['z'] / 1000) if not math.isnan(bottom_left_spatial['z']) else "--"), (bottomLeft[0] - 50, bottomLeft[1] + 30))

                y_muliplication_factor = -1  # Hack for Coordinate System Unsync between OAK and Yanthra TODO

                ContentTxt += "{0:.3f} ".format(top_left_spatial['x'] / 1000)
                ContentTxt += "{0:.3f} ".format(top_left_spatial['y'] * y_muliplication_factor / 1000)
                ContentTxt += "{0:.3f} ".format(top_left_spatial['z'] / 1000) + "\n"
                ContentTxt += "{0:.3f} ".format(top_right_spatial['x'] / 1000)
                ContentTxt += "{0:.3f} ".format(top_right_spatial['y'] * y_muliplication_factor / 1000)
                ContentTxt += "{0:.3f} ".format(top_right_spatial['z'] / 1000) + "\n"
                ContentTxt += "{0:.3f} ".format(bottom_right_spatial['x'] / 1000)
                ContentTxt += "{0:.3f} ".format(bottom_right_spatial['y'] * y_muliplication_factor / 1000)
                ContentTxt += "{0:.3f} ".format(bottom_right_spatial['z'] / 1000) + "\n"
                ContentTxt += "{0:.3f} ".format(bottom_left_spatial['x'] / 1000)
                ContentTxt += "{0:.3f} ".format(bottom_left_spatial['y'] * y_muliplication_factor / 1000)
                ContentTxt += "{0:.3f} ".format(bottom_left_spatial['z'] / 1000) + "\n"

                CottonDetailsTxt += "0 0 " + "{0:.3f} ".format(top_left_spatial['x'] / 1000)
                CottonDetailsTxt += "{0:.3f} ".format(top_left_spatial['y'] * y_muliplication_factor / 1000)
                CottonDetailsTxt += "{0:.3f} ".format(top_left_spatial['z'] / 1000) + "\n"
                CottonDetailsTxt += "0 0 " + "{0:.3f} ".format(top_right_spatial['x'] / 1000)
                CottonDetailsTxt += "{0:.3f} ".format(top_right_spatial['y'] * y_muliplication_factor / 1000)
                CottonDetailsTxt += "{0:.3f} ".format(top_right_spatial['z'] / 1000) + "\n"
                CottonDetailsTxt += "0 0 " + "{0:.3f} ".format(bottom_right_spatial['x'] / 1000)
                CottonDetailsTxt += "{0:.3f} ".format(bottom_right_spatial['y'] * y_muliplication_factor / 1000)
                CottonDetailsTxt += "{0:.3f} ".format(bottom_right_spatial['z'] / 1000) + "\n"
                CottonDetailsTxt += "0 0 " + "{0:.3f} ".format(bottom_left_spatial['x'] / 1000)
                CottonDetailsTxt += "{0:.3f} ".format(bottom_left_spatial['y'] * y_muliplication_factor / 1000)
                CottonDetailsTxt += "{0:.3f} ".format(bottom_left_spatial['z'] / 1000) + "\n"

        cv2.imwrite(ArucoDetectorOutputImage, frameRight)
        centroid_file.write(ContentTxt)
        CottonDetailsTxtFile.write(CottonDetailsTxt)
        print("Wrote to centroid.txt")
        print(ContentTxt)
        break

centroid_file.close()
CottonDetailsTxtFile.close()
