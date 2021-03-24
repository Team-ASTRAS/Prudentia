from picamera import PiCamera
import time
import numpy as np
import cv2
import base64

class CameraSingleton:
    camera = {}

    def __init__(self):
        #Camera setup.
        #Set resolution, other settings
        self.camera = PiCamera()
        self.camera.resolution = (320,240) # if this changes then the FOV math below needs to be updated
        self.camera.framerate = 24
        time.sleep(2)
        output = np.empty((240,320,3), dtype=np.uint8)
        self.camera.capture(output,'bgr')
        mystring = base64.b64encode(output)
        print(mystring) 
        
    def getPicture(self):
        #This now gets a picture
        
        self.camera.start_preview()
        time.sleep(10) # if you just want to see stuffs
        for i in range(5):
            self.camera.capture('/home/pi/Desktop/Prudentia/rPi Scripts/targetss/image%s.bmp' % i)
        self.camera.stop_preview()

    
Camera = CameraSingleton()