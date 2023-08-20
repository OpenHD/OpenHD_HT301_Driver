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
import threading
from dronekit import connect
from v4l2 import (
    v4l2_format, VIDIOC_G_FMT, V4L2_BUF_TYPE_VIDEO_OUTPUT, V4L2_PIX_FMT_RGB24,
    V4L2_FIELD_NONE, VIDIOC_S_FMT
)

########### init ###########

rc_channel = 800 # aux channel 4 = channel 8, for cycle colormaps
ch_state = False
prev_ch_state = False
mavlink = False

VIDEO_IN = "/dev/video0"
VIDEO_OUT = "/dev/video7"
VID_WIDTH = 256
VID_HEIGHT = 196

# list of used colormaps. all available colormaps here: https://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html#ga9a805d8262bcbe273f16be9ea2055a65
colormaps = [cv2.COLORMAP_PINK, -1, cv2.COLORMAP_INFERNO, -1, cv2.COLORMAP_TURBO, -1] # add -1 for dde algorithm
selectedmap = 0 # selected map on startup. default is 0

flipped_camera = True
draw_temp = True
calibration_offset = True

############################

# cycle colormaps
def cyclecolormaps():
    global selectedmap
    if selectedmap < len(colormaps) - 1:
        selectedmap += 1
    else:
        selectedmap = 0
    print("Switching to colormap: ", selectedmap + 1)

def channel_listener(self, name, message):
    global rc_channel
    rc_channel = message.chan8_raw

def mavlink_connect():
    try:
        vehicle = connect("/dev/serial0", wait_ready=False, baud=115200) # serial connection to FC
        vehicle.add_message_listener('RC_CHANNELS', channel_listener)
        mavlink = True
    except Exception as e:
        print(e)
        mavlink = False

def main():

    ########### camera #############

    # create camera object. initial calibration will be executed by constructor
    cap = ht301_hacklib.T2SPLUS()

    # read calibration array from file
    offset = np.load("noise_pattern_calibration.npy")
    print("Info: calibration file read")

    # create a Contrast Limited Adaptive Histogram Equalization object. default: 5.0, (6, 6)
    clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(6, 6))
    print("Info: histogram created")

    # open v4l2 output device
    videooutput = os.open(VIDEO_OUT, os.O_RDWR)
    print("Info: v4l2 opened")

    # configure parameters for output device
    vid_format = v4l2_format()
    vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT
    if fcntl.ioctl(videooutput, VIDIOC_G_FMT, vid_format) < 0:
        print("ERROR: unable to get video format!")
        return -1

    framesize = VID_WIDTH * VID_HEIGHT * 3
    vid_format.fmt.pix.width = VID_WIDTH
    vid_format.fmt.pix.height = VID_HEIGHT
    print("Info: configure framesize done")

    # pixel format
    vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24
    vid_format.fmt.pix.sizeimage = framesize
    vid_format.fmt.pix.field = V4L2_FIELD_NONE
    print("Info: configure pixelformat done")

    if fcntl.ioctl(videooutput, VIDIOC_S_FMT, vid_format) < 0:
        print("ERROR: unable to set video format!")
        return -1
        
    print("Info: video format set")

    ################################

    # video loop
    while(True):

    print("Info: main loop begin")

        # cycle only one map per button press
        if rc_channel > 1800:
            ch_state = True
            if prev_ch_state == False:
                cyclecolormaps()
        else:
            ch_state = False
        prev_ch_state = ch_state

        ret, frame = cap.read() # read frame from thermal camera
        print("Info: read frame")
        info, lut = cap.info() # get hottest and coldest spot and its temperatures
        print("Info: read temperatures")


        # automatic gain control
        frame = frame.astype(np.float32)
        if calibration_offset == True: # apply calibration offset
            frame = frame - offset + np.mean(offset)
        frame = (255*((frame - frame.min())/(frame.max()-frame.min()))).astype(np.uint8) # cast frame to values from 0 to 255
        if colormaps[selectedmap] == -1:
            # digital detail enhancement algorithm
            framebuffer = frame
            clahe.apply(framebuffer, frame)
            frame = cv2.applyColorMap(frame, colormaps[selectedmap - 1])
        else:
            # apply colormap
            frame = cv2.applyColorMap(frame, colormaps[selectedmap])

        if flipped_camera == True: # rotate the temperature points with the image if the thermal camera is mounted upside down
            (coldx, coldy) = info['Tmin_point']
            (warmx, warmy) = info['Tmax_point']
            coldestpoint = (VID_WIDTH - coldx, VID_HEIGHT - coldy)
            warmestpoint = (VID_WIDTH - warmx, VID_HEIGHT - warmy)
            frame = cv2.flip(frame, -1) # flip the frame
        else:
            coldestpoint = info['Tmin_point']
            warmestpoint = info['Tmax_point']

        if draw_temp: # color: BGR
            utils.drawTemperature(frame, coldestpoint, info['Tmin_C'], (200, 200, 200)) # coldest spot
            utils.drawTemperature(frame, warmestpoint, info['Tmax_C'], (30, 30, 30)) # hottest spot
            #utils.drawTemperature(frame, info['Tcenter_point'], info['Tcenter_C'], (0, 255, 255)) # center spot

        # output
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # convert opencv bgr to rgb for the v4l2 pixelformat
        written = os.write(videooutput, frame.data) # write frame to output device

    cap.release()
    if mavlink:
        vehicle.close()
    return 0

if __name__ == "__main__":
    # create thread for main loop and mavlink connection loop
    mainloop = threading.Thread(target=main)
    mavlinkloop = threading.Thread(target=mavlink_connect)
    # start threads
    mainloop.start()
    mavlinkloop.start()
