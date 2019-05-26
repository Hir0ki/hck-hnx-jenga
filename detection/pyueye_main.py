#!/usr/bin/env python

from pyueye import ueye
from detection.pyueye_camera import Camera
from detection.pyueye_utils import FrameThread

import time

import cv2
import numpy as np



class Picture():

    def __init__(self):
        self.cam = Camera()
        self.cam.init()

        self.cam.alloc()
        self.cam.capture_video()


    def get_next_frame(self):
        Frame = FrameThread(self.cam)
        rows = Frame.run()
        
        new_rows = []
        for row in rows: 
            new_row = []
            for entry in row:
                new_row.append(True)
            new_rows.append(new_row)

        return new_rows

        

    def shutdown(self):

        cv2.destroyAllWindows()
        self.cam.stop_video()
        self.cam.exit()
