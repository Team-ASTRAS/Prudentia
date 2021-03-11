from picamera import PiCamera
from time import sleep
import cv2
import numpy as np
import argparse
import imutils
import math
from matplotlib import pyplot as plt

class CameraSingleton:
    camera = {}

    def __init__(self):
        #Camera setup.
        #Set resolution, other settings
        self.camera = PiCamera()
        self.camera.resolution = (540,500)
        self.camera.framerate = 20
        
        
        # Target Calculation 
        template = cv2.imread('target.bmp')
        template_gray = cv2.imread('target.bmp',0)
        img = cv2.imread('/home/pi/Desktop/Prudentia/rPi Scripts/images/image3.bmp') # reads in image taken
        img_gray = cv2.imread('/home/pi/Desktop/Prudentia/rPi Scripts/images/image3.bmp',0) # makes it gray
        res = cv2.matchTemplate(img_gray,template_gray,cv2.TM_CCOEFF_NORMED)
        threshold = 0.9
        loc = np.where(res>=threshold)
        loc_x = np.median(loc[1]) # middle of target identified
        loc_y = np.median(loc[0]) # middle of target identified
        print(loc_x)
        w, h = template_gray.shape[::-1]
        cv2.circle(img,(270,250),5,[0,0,255],5) # Put dot in center of picture
        for pt in zip(*loc[::-1]):
            cv2.rectangle(img,pt,(pt[0]+w,pt[1]+h),[0,255,255],1) # draw target borders on image     
            cv2.circle(img,(int(loc_x+(w/2)),int(loc_y+(h/2))),5,[0,0,255],5) # draw center point on image
            pixel_distance = math.sqrt(((270-(int(loc_x+(w/2))))**2)+((250-(int(loc_y+(h/2))))**2))
        distance = np.mean(pixel_distance) #average distance in pixels from targets center to image center
        cv2.line(img,(270,250),(int(loc_x+(w/2)),int(loc_y+(h/2))),[0,0,255],2)
    
        cv2.imshow('detected',img) # show image with target identified and center dot
        cv2.imshow('target',template) # show target image 
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        # Calculate shit
            # these following calculations assume a linear relationship between pixel distance and theta,
            # not a great assumption but it'll work!
        x_pixels = int(270-int(loc_x+(w/2)))
        FOV_X = 62.6 # deg
        if x_pixels > 0:
            theta_x = -(x_pixels*(31.3/270)) # assumes to the left is negative
        else:
            theta_x = (x_pixels*(31.3/270)) # assumes to the right is positive
        
        y_pixels = int(250-int(loc_y+(h/2)))
        FOV_y = 48.8 # deg
        if y_pixels > 0:
            theta_y = -(y_pixels*(24.4/250)) # assumes up is positive
        else:
            theta_y = (y_pixels*(24.4/250)) # assumes down in negative
        pass
        
    def getPicture(self):
        #This now gets a picture
        # start camera preview and take initial 5 images
        #self.camera.start_preview()
        #for i in range(5):
            #self.camera.capture('/home/pi/Desktop/Prudentia/rPi Scripts/images/image%s.bmp' % i)
        #self.camera.stop_preview()
        pass
    

    def findLocation(self, picture):
        #Return screen position based on target in picture
        #Or, return None if no target is found
        #This image processing algorithm is going to have to be fast. Hopefully IR contrast helps enough
         
        pass

    def translatePosition(self, location):
        #This function converts a screen location to a relative attitude.
        # LMAO hard
        
        
        pass
    
Camera = CameraSingleton()
Camera.getPicture()                                                                                                                            