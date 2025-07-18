#!/usr/bin/env python3

import cv2
import depthai as dai
import pickle as pkl
import numpy as np
import os
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
from cv2 import aruco
from calc import HostSpatialsCalc
from utility import *
import math

# Start defining a pipeline
pipeline = dai.Pipeline()

# Define sources - two mono (grayscale) cameras and one color camera
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
stereo.initialConfig.setConfidenceThreshold(255)  # Updated to fix deprecation warning
stereo.setLeftRightCheck(True)  # Enable LR-check for RGB alignment
stereo.setSubpixel(False)
stereo.setOutputDepth(True)
stereo.setOutputRectified(False)

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
stereo.depth.link(xoutDepth.input)

device = dai.Device(pipeline)
device.startPipeline()

# Output queues
depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
qColor = device.getOutputQueue(name="color", maxSize=4, blocking=False)

# ArUco declaration
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()
text = TextHelper()
hostSpatials = HostSpatialsCalc(device)
delta = 2
hostSpatials.setDeltaRoi(delta)
centroid_file = open('centroid.txt', 'w')

while True:
    inDepth = depthQueue.get().getFrame()  # Blocking call, waits for depth data
    inColor = qColor.tryGet()

    if inColor is None:
        print("Error: No color frame received. Check camera connection.")
        continue

    frameColor = inColor.getCvFrame()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frameColor, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frameColor.copy(), corners, ids)

    if len(corners) > 0:
        # Flatten the ArUco IDs list
        ids = ids.flatten()
        # Loop over the detected ArUco corners
        for (markerCorner, markerID) in zip(corners, ids):
            # Extract the marker corners (top-left, top-right, bottom-right, bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # Convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # Calculate spatial coordinates
            top_left_spatial, centroid = hostSpatials.calc_spatials(inDepth, (topLeft[0], topLeft[1]))
            top_right_spatial, centroid = hostSpatials.calc_spatials(inDepth, (topRight[0], topRight[1]))
            bottom_right_spatial, centroid = hostSpatials.calc_spatials(inDepth, (bottomRight[0], bottomRight[1]))
            bottom_left_spatial, centroid = hostSpatials.calc_spatials(inDepth, (bottomLeft[0], bottomLeft[1]))

            # Draw rectangles and text for each corner
            text.rectangle(frameColor, (topLeft[0]-delta, topLeft[1]-delta), (topLeft[0]+delta, topLeft[1]+delta))
            text.putText(frameColor, "X: " + ("{:.1f}".format(top_left_spatial['x']/1000) if not math.isnan(top_left_spatial['x']) else "--") +
                         " Y: " + ("{:.1f}".format(top_left_spatial['y']/1000) if not math.isnan(top_left_spatial['y']) else "--") +
                         " Z: " + ("{:.1f}".format(top_left_spatial['z']/1000) if not math.isnan(top_left_spatial['z']) else "--"),
                         (topLeft[0] - 50, topLeft[1] - 30))

            text.rectangle(frameColor, (topRight[0]-delta, topRight[1]-delta), (topRight[0]+delta, topRight[1]+delta))
            text.putText(frameColor, "X: " + ("{:.1f}".format(top_right_spatial['x']/1000) if not math.isnan(top_right_spatial['x']) else "--") +
                         " Y: " + ("{:.1f}".format(top_right_spatial['y']/1000) if not math.isnan(top_right_spatial['y']) else "--") +
                         " Z: " + ("{:.1f}".format(top_right_spatial['z']/1000) if not math.isnan(top_right_spatial['z']) else "--"),
                         (topRight[0] + 10, topRight[1] + 10))

            text.rectangle(frameColor, (bottomRight[0]-delta, bottomRight[1]-delta), (bottomRight[0]+delta, bottomRight[1]+delta))
            text.putText(frameColor, "X: " + ("{:.1f}".format(bottom_right_spatial['x']/1000) if not math.isnan(bottom_right_spatial['x']) else "--") +
                         " Y: " + ("{:.1f}".format(bottom_right_spatial['y']/1000) if not math.isnan(bottom_right_spatial['y']) else "--") +
                         " Z: " + ("{:.1f}".format(bottom_right_spatial['z']/1000) if not math.isnan(bottom_right_spatial['z']) else "--"),
                         (bottomRight[0] + 10, bottomRight[1] + 10))

            text.rectangle(frameColor, (bottomLeft[0]-delta, bottomLeft[1]-delta), (bottomLeft[0]+delta, bottomLeft[1]+delta))
            text.putText(frameColor, "X: " + ("{:.1f}".format(bottom_left_spatial['x']/1000) if not math.isnan(bottom_left_spatial['x']) else "--") +
                         " Y: " + ("{:.1f}".format(bottom_left_spatial['y']/1000) if not math.isnan(bottom_left_spatial['y']) else "--") +
                         " Z: " + ("{:.1f}".format(bottom_left_spatial['z']/1000) if not math.isnan(bottom_left_spatial['z']) else "--"),
                         (bottomLeft[0] - 50, bottomLeft[1] + 30))

        # Resize the frame for display to 416x416
        frameColor_resized = cv2.resize(frameColor, (416, 416))
        cv2.imshow("aruco_corner_spatials", frameColor_resized)

    key = cv2.waitKey(1)
    if key == ord('q'):
        if 'top_left_spatial' in locals():
            content = f"{top_left_spatial['x']/1000} {top_left_spatial['y']/1000} {top_left_spatial['z']/1000}\n" \
                      f"{top_right_spatial['x']/1000} {top_right_spatial['y']/1000} {top_right_spatial['z']/1000}\n" \
                      f"{bottom_right_spatial['x']/1000} {bottom_right_spatial['y']/1000} {bottom_right_spatial['z']/1000}\n" \
                      f"{bottom_left_spatial['x']/1000} {bottom_left_spatial['y']/1000} {bottom_left_spatial['z']/1000}"
            centroid_file.write(content)
        centroid_file.close()
        break
