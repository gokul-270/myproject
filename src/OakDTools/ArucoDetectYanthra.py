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
colorCam = pipeline.createColorCamera()
stereo = pipeline.createStereoDepth()
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
xoutDepth = pipeline.createXLinkOut()
xoutColor = pipeline.createXLinkOut()

xoutDepth.setStreamName("depth")
xoutColor.setStreamName('color')

# MonoCamera
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# ColorCamera
colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
colorCam.setBoardSocket(dai.CameraBoardSocket.RGB)
colorCam.setInterleaved(False)
colorCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
colorCam.video.link(xoutColor.input)

# StereoDepth
stereo.initialConfig.setConfidenceThreshold(255)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)
stereo.setExtendedDisparity(False)
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
qColor = device.getOutputQueue(name="color", maxSize=4, blocking=False)

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
    inColor = qColor.tryGet()

    if inColor is None:
        print("Error: OAK-D not capturing color frames")
        print("Error: Check Camera")
        continue

    if inColor is not None:
        frameColor = inColor.getCvFrame()
        if cv2.imwrite(ArucoDetectorInputImage, frameColor):
            print("ArucoDetectYanthra.py: Wrote ~/pragati/inputs/ArucoInputImage.jpg")
        else:
            print("ArucoDetectYanthra.py: Not able to write ~/pragati/inputs/ArucoInputImage.jpg")

        corners, ids, rejectedImgPoints = aruco.detectMarkers(frameColor, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frameColor.copy(), corners, ids)

        print("No Of Aruco Corner Found: {}".format(len(corners)))
        if len(corners) <= 0:
            print("ArucoMarkerDetect: Failed to detect corners")
            print("ArucoDetectYanthra.py: Error Exiting Program")
            continue

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

                text.rectangle(frameColor, (topLeft[0] - delta, topLeft[1] - delta), (topLeft[0] + delta, topLeft[1] + delta))
                text.putText(frameColor, "X: " + ("{:.3f}".format(top_left_spatial['x'] / 1000) if not math.isnan(top_left_spatial['x']) else "--") + " Y: " + ("{:.3f}".format(top_left_spatial['y'] / 1000) if not math.isnan(top_left_spatial['y']) else "--") + " Z: " + ("{:.3f}".format(top_left_spatial['z'] / 1000) if not math.isnan(top_left_spatial['z']) else "--"), (topLeft[0] - 50, topLeft[1] - 30))

                text.rectangle(frameColor, (topRight[0] - delta, topRight[1] - delta), (topRight[0] + delta, topRight[1] + delta))
                text.putText(frameColor, "X: " + ("{:.3f}".format(top_right_spatial['x'] / 1000) if not math.isnan(top_right_spatial['x']) else "--") + " Y: " + ("{:.3f}".format(top_right_spatial['y'] / 1000) if not math.isnan(top_right_spatial['y']) else "--") + " Z: " + ("{:.3f}".format(top_right_spatial['z'] / 1000) if not math.isnan(top_right_spatial['z']) else "--"), (topRight[0] + 10, topRight[1] + 10))

                text.rectangle(frameColor, (bottomRight[0] - delta, bottomRight[1] - delta), (bottomRight[0] + delta, bottomRight[1] + delta))
                text.putText(frameColor, "X: " + ("{:.3f}".format(bottom_right_spatial['x'] / 1000) if not math.isnan(bottom_right_spatial['x']) else "--") + " Y: " + ("{:.3f}".format(bottom_right_spatial['y'] / 1000) if not math.isnan(bottom_right_spatial['y']) else "--") + " Z: " + ("{:.3f}".format(bottom_right_spatial['z'] / 1000) if not math.isnan(bottom_right_spatial['z']) else "--"), (bottomRight[0] + 10, bottomRight[1] + 10))

                text.rectangle(frameColor, (bottomLeft[0] - delta, bottomLeft[1] - delta), (bottomLeft[0] + delta, bottomLeft[1] + delta))
                text.putText(frameColor, "X: " + ("{:.3f}".format(bottom_left_spatial['x'] / 1000) if not math.isnan(bottom_left_spatial['x']) else "--") + " Y: " + ("{:.3f}".format(bottom_left_spatial['y'] / 1000) if not math.isnan(bottom_left_spatial['y']) else "--") + " Z: " + ("{:.3f}".format(bottom_left_spatial['z'] / 1000) if not math.isnan(bottom_left_spatial['z']) else "--"), (bottomLeft[0] - 50, bottomLeft[1] + 30))

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

        cv2.imwrite(ArucoDetectorOutputImage, frameColor)
        centroid_file.write(ContentTxt)
        CottonDetailsTxtFile.write(CottonDetailsTxt)
        print("Wrote to centroid.txt")
        print(ContentTxt)
        break

centroid_file.close()
CottonDetailsTxtFile.close()

