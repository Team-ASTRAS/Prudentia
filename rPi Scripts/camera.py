from picamera import PiCamera
import cv2
import numpy as np
import math
import time

class CameraSingleton:

    def __init__(self):
        #Camera setup.
        #Set resolution, other settings
        self.camera = PiCamera()
        self.camera.resolution = (540,500) # if this changes then the FOV math below needs to be updated
        self.camera.framerate = 15
        
    def getPicture(self):
        #This now gets a picture
        
        #self.camera.start_preview()
        #time.sleep(20) # if you just want to see stuffs
        for i in range(1):
            self.camera.capture('/home/pi/Desktop/Prudentia/rPi Scripts/images/image%s.bmp' % i)
        
            
    def findLocation(self):
        #Return screen position based on target in picture
        #Or, return None if no target is found
        #This image processing algorithm is going to have to be fast. Hopefully IR contrast helps enough
        
        startTime = time.time()       
        # Target Calculation 
        template_gray = cv2.imread('target_00.bmp',0) # 0 makes it read in gray instead of color
        img = cv2.imread('/home/pi/Desktop/Prudentia/rPi Scripts/images/image3.bmp') # reads in image taken
        img_gray = cv2.imread('/home/pi/Desktop/Prudentia/rPi Scripts/images/image3.bmp',0) # makes it gray
        res = cv2.matchTemplate(img_gray,template_gray,cv2.TM_CCOEFF_NORMED) #matches using cv2 in gray scale images
        threshold = 0.5
        loc = np.where(res>=threshold)
        loc_x = np.median(loc[1]) # middle of target identified in X
        loc_y = np.median(loc[0]) # middle of target identified in Y
        w, h = template_gray.shape[::-1] # find size of template
        
        # Everything in lines 44-52 can be commented out for faster calculations (about 1/10th of a second) 
        cv2.circle(img,(270,250),5,[0,0,255],5) # Put dot in center of picture
        for pt in zip(*loc[::-1]):
            cv2.rectangle(img,pt,(pt[0]+w,pt[1]+h),[0,255,255],1) # draw target borders on image     
            cv2.circle(img,(int(loc_x+(w/2)),int(loc_y+(h/2))),5,[0,0,255],5) # draw center point on image
            pixel_distance = math.sqrt(((270-(int(loc_x+(w/2))))**2)+((250-(int(loc_y+(h/2))))**2))
        #distance = np.mean(pixel_distance) # average distance in pixels from targets center to image center
        
        cv2.line(img,(270,250),(int(loc_x+(w/2)),int(loc_y+(h/2))),[0,0,255],2)
        cv2.imwrite("detected.bmp",img)
        #cv2.imshow('detected',img) # show image with target identified and center dot
        #cv2.imshow('target',template) # show target image 
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        
        # these following calculations assume a linear relationship between pixel distance and theta,
        # not a great assumption but it'll work!
        x_pixels = int(270-int(loc_x+(w/2))) # half of the resolution in X subtracted from the targets X location
        FOV_x = 62.6 # deg, ratio looked up for field of view for the resolution being used on Camera
        if x_pixels > 0:
            theta_x = -(x_pixels*((FOV_x/2)/270)) # assumes to the left is negative
        else:
            theta_x = (x_pixels*((FOV_x/2)/270)) # assumes to the right is positive
        
        y_pixels = int(250-int(loc_y+(h/2))) # half of the resolution in Y subtracted from the targets Y location
        FOV_y = 48.8 # deg, ratio looked up for field of view for the resolution being used on Camera
        if y_pixels > 0:
            theta_y = (y_pixels*((FOV_y/2)/250)) # assumes up is positive
        else:
            theta_y = -(y_pixels*((FOV_y/2)/250)) # assumes down in negative
    
        print(theta_x)
        print(theta_y)
        
        endTime = time.time()
        print("Find location time taken: %s" % (endTime - startTime))
        
        #self.camera.stop_preview()
        return [theta_x,theta_y]
    
    def translatePosition(self):
        #This function converts a screen location to a relative attitude.
        # Does this above
        pass
    
Camera = CameraSingleton()
Camera.getPicture()
Camera.findLocation()