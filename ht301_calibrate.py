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

def main():

    # create camera object. initial calibration will be executed by constructor
    cap = ht301_hacklib.HT301()

    ret, frame = cap.read() # read frame from thermal camera
    time.sleep(3)
    ret, frame = cap.read() # read frame from thermal camera
    offset = frame.astype(np.float32)

    np.save("noise_pattern_calibration.npy", offset)
    print("Calibration successfull!")
    
    cap.release()
    return 0

if __name__ == "__main__":
    sys.exit(main())
