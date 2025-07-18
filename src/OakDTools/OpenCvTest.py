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

frame =  cv2.imread("ArucoDetectorInputImage.jpg" )
cv2.imwrite("/tmp/ArucoDetectorInputImage.jpg" ,frame)
