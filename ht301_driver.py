#!/usr/bin/python3
import math
import numpy as np
import cv2
import math
import ht301_hacklib
import utils
import time
import os
import sys
import cv2
import fcntl
#import asyncio
#from mavsdk import System
from dronekit import connect
from v4l2 import (
    v4l2_format, VIDIOC_G_FMT, V4L2_BUF_TYPE_VIDEO_OUTPUT, V4L2_PIX_FMT_RGB24,
    V4L2_FIELD_NONE, VIDIOC_S_FMT
)

########### init ###########

#vehicle = connect("tcp://localhost:5761", wait_ready=False) # address of the openhd mavlink router

rc_channel = '8' # aux channel 4 = channel 8, for cycle colormaps
ch_state = False
prev_ch_state = False

VIDEO_IN = "/dev/video0"
VIDEO_OUT = "/dev/video5"
VID_WIDTH = 384
VID_HEIGHT = 288

flipped_camera = True
draw_temp = True

# list of used colormaps. all available colormaps here: https://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html#ga9a805d8262bcbe273f16be9ea2055a65
colormaps = [cv2.COLORMAP_BONE, cv2.COLORMAP_PINK, cv2.COLORMAP_INFERNO, cv2.COLORMAP_JET] # add -1 for dde algorithm
selectedmap = 1 # selected map on startup. default is 0

# create a Contrast Limited Adaptive Histogram Equalization object.
clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(4,4))

############################

# cycle colormaps
def cyclecolormaps():
    if selectedmap < len(colormaps) - 1:
        selectedmap += 1
    else:
        selectedmap = 0

def main():

    ########### camera #############
    
    # create camera object. initial calibration will be executed by constructor
    cap = ht301_hacklib.HT301()
    
    # open v4l2 output device
    videooutput = os.open(VIDEO_OUT, os.O_RDWR)

    # configure params for output device
    vid_format = v4l2_format()
    vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT
    if fcntl.ioctl(videooutput, VIDIOC_G_FMT, vid_format) < 0:
        print("ERROR: unable to get video format!")
        return -1

    framesize = VID_WIDTH * VID_HEIGHT * 3
    vid_format.fmt.pix.width = VID_WIDTH
    vid_format.fmt.pix.height = VID_HEIGHT

    # pixel format
    vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24
    vid_format.fmt.pix.sizeimage = framesize
    vid_format.fmt.pix.field = V4L2_FIELD_NONE

    if fcntl.ioctl(videooutput, VIDIOC_S_FMT, vid_format) < 0:
        print("ERROR: unable to set video format!")
        return -1

    ################################

    while(True):
        # cycle only one map per button press
        #if vehicle.channels[rc_channel] > 1800:
        #    ch_state = True
        #    if prev_ch_state == False:
        #        cyclecolormaps()
        #else:
        #    ch_state = False
        #prev_ch_state = ch_state


        ret, frame = cap.read() # read frame from thermal camera
        orig_frame = frame
        info, lut = cap.info() # get hottest and coldest spot and its temperatures

        if selectedmap != 0: # do not apply anything when 0 is selected. original frame
            # sketchy auto-exposure
            frame = frame.astype(np.float32)
            #frame = 255*(frame - frame.min())/(frame.max()-frame.min()).astype(np.uint8)
            frame -= frame.min()
            frame /= frame.max()
            frame = (np.clip(frame, 0, 1)*255).astype(np.uint8) # deprecated

            if colormaps[selectedmap] == -1:
                # dde algorithm implementation
                # coming soon...
                #frame = clahe.apply(frame)
            else:
                # apply colormap
                #frame = (255-frame) # invert range if colormap is inverted
                frame = cv2.applyColorMap(frame, colormaps[selectedmap])

        if flipped_camera: # rotate the temperature points with the image if the thermal camera is mounted upside down
            (coldx, coldy) = info['Tmin_point']
            (warmx, warmy) = info['Tmax_point']
            coldestpoint = (VID_WIDTH - coldx, VID_HEIGHT - coldy)
            warmestpoint = (VID_WIDTH - warmx, VID_HEIGHT - warmy)
            
            frame = cv2.flip(frame, -1) # flip the frame
        else:
            coldestpoint = info['Tmin_point']
            warmestpoint = info['Tmax_point']

        if draw_temp:
            utils.drawTemperature(frame, coldestpoint, info['Tmin_C'], (0,0,255)) # coldest spot
            utils.drawTemperature(frame, warmestpoint, info['Tmax_C'], (255,0,0)) # hottest spot
            #utils.drawTemperature(frame, info['Tcenter_point'], info['Tcenter_C'], (255,255,0)) # center spot

        # write frame to output device
        written = os.write(videooutput, frame.data)

    cap.release()
    #vehicle.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())


