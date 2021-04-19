# Web streaming example
# Source code from the official PiCamera package
# http://picamera.readthedocs.io/en/latest/recipes2.html#web-streaming

import io
import picamera
import logging
import socketserver
import http.client
import time
import cv2
import math
import numpy as np
from threading import Condition, Thread
from http import server


PAGE="""\
<html>
<head>
</head>
<body>
<center><img src="stream.mjpg"></center>
</body>
</html>
"""
#<center><h1>Raspberry Pi - Surveillance Camera</h1></center>
class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
               'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

output = StreamingOutput()

camera = picamera.PiCamera()

resolution = 500

camera.resolution = (resolution, resolution) # if this changes then the FOV math below needs to be updated
camera.framerate = 24
#camera.shutter_speed = 8000
def startCamServer():
    #Uncomment the next line to change your Pi's Camera rotation (in degrees)
    #camera.rotation = 90
    camera.start_recording(output, format='mjpeg')
    try:
        address = ('', 8000)
        server = StreamingServer(address, StreamingHandler)
        server.serve_forever()
    
    finally:
        camera.stop_recording()
            
def takePictures():
    while True:
        time.sleep(1)
        #imgOutput = np.empty((512,512,3), dtype=np.uint8)
        camera.capture('/home/pi/Desktop/Prudentia/rPi Scripts/images/image3.bmp', use_video_port=True)
        print(findLocation())
        
def findLocation():
    #Return screen position based on target in picture
    #Or, return None if no target is found
    #This image processing algorithm is going to have to be fast. Hopefully IR contrast helps enough
    
    startTime = time.time()       
    # Target Calculation 
    template_gray = cv2.imread('circle_target.bmp',0) # 0 makes it read in gray instead of color
    img = cv2.imread('/home/pi/Desktop/Prudentia/rPi Scripts/images/image3.bmp') # reads in image taken
    img_gray = cv2.imread('/home/pi/Desktop/Prudentia/rPi Scripts/images/image3.bmp',0) # makes it gray
    #imgOutput = cv2.cvtColor(imgOutput, cv2.COLOR_BGR2GRAY)
    res = cv2.matchTemplate(img_gray,template_gray,cv2.TM_CCOEFF_NORMED) #matches using cv2 in gray scale images
    threshold = 0.8
    loc = np.where(res>=threshold)
    
        
    if(loc[0].size > 0):
        
        loc_x = np.median(loc[1]) # middle of target identified in X
        loc_y = np.median(loc[0]) # middle of target identified in Y
        w, h = template_gray.shape[::-1] # find size of template
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
        
        endTime = time.time()
        print("Find location time taken: %s" % (endTime - startTime))
        return [theta_x,theta_y,x_pixels, y_pixels]
    else:
        #Nothing was found
        endTime = time.time()
        print("Find location time taken: %s" % (endTime - startTime))
        
        return [None, None]
        
if __name__ == "__main__":
    camThread = Thread(target=startCamServer)
    camThread.start()
    takePictures()
    
    camThread.join() #Wait for thread to finish running
    
    
    
    
    
    
    
    
    
#BACKUP OF OLD CODE:
    # from picamera import PiCamera
# import cv2
# import numpy as np
# import math
# import time
# import base64
# import io
# 
# class CameraSingleton:
# 
#     def __init__(self):
#         #Camera setup.
#         #Set resolution, other settings
#         self.camera = PiCamera()
#         self.camera.resolution = (320,240) # if this changes then the FOV math below needs to be updated
#         self.camera.framerate = 120
#         self.camera.shutter_speed = 8000
#         
#         self.camera.start_preview()
#         time.sleep(20)
#         self.camera.stop_preview()
#         
#     def getPictureString(self):
#         #This returns a base64 image string
#         t0 = time.time()
#         output = np.empty((240,320,3), dtype=np.uint8)
#         
#         t1 = time.time()
#         self.camera.capture(output, 'bgr', use_video_port=True)
#         
#         t2 = time.time()
#         b = base64.b64encode(output)
#         
#         t3 = time.time()
#         s = str(b)
#         
#         t4 = time.time()
#         print("T1-T0: %s" % (t1-t0))
#         print("T2-T1: %s" % (t2-t1))
#         print("T3-T2: %s" % (t3-t2))
#         print("T4-T3: %s" % (t4-t3))
#         return s
#     
#     def findLocation(self):
#         #Return screen position based on target in picture
#         #Or, return None if no target is found
#         #This image processing algorithm is going to have to be fast. Hopefully IR contrast helps enough
#         
#         startTime = time.time()       
#         # Target Calculation 
#         template_gray = cv2.imread('target_00.bmp',0) # 0 makes it read in gray instead of color
#         img = cv2.imread('/home/pi/Desktop/Prudentia/rPi Scripts/images/image3.bmp') # reads in image taken
#         img_gray = cv2.imread('/home/pi/Desktop/Prudentia/rPi Scripts/images/image3.bmp',0) # makes it gray
#         res = cv2.matchTemplate(img_gray,template_gray,cv2.TM_CCOEFF_NORMED) #matches using cv2 in gray scale images
#         threshold = 0.5
#         loc = np.where(res>=threshold)
#         loc_x = np.median(loc[1]) # middle of target identified in X
#         loc_y = np.median(loc[0]) # middle of target identified in Y
#         w, h = template_gray.shape[::-1] # find size of template
#         
#         # Everything in lines 44-52 can be commented out for faster calculations (about 1/10th of a second) 
#         cv2.circle(img,(270,250),5,[0,0,255],5) # Put dot in center of picture
#         for pt in zip(*loc[::-1]):
#             cv2.rectangle(img,pt,(pt[0]+w,pt[1]+h),[0,255,255],1) # draw target borders on image     
#             cv2.circle(img,(int(loc_x+(w/2)),int(loc_y+(h/2))),5,[0,0,255],5) # draw center point on image
#             pixel_distance = math.sqrt(((270-(int(loc_x+(w/2))))**2)+((250-(int(loc_y+(h/2))))**2))
#         #distance = np.mean(pixel_distance) # average distance in pixels from targets center to image center
#         
#         cv2.line(img,(270,250),(int(loc_x+(w/2)),int(loc_y+(h/2))),[0,0,255],2)
#         cv2.imwrite("detected.bmp",img)
#         #cv2.imshow('detected',img) # show image with target identified and center dot
#         #cv2.imshow('target',template) # show target image 
#         #cv2.waitKey(0)
#         #cv2.destroyAllWindows()
#         
#         # these following calculations assume a linear relationship between pixel distance and theta,
#         # not a great assumption but it'll work!
#         x_pixels = int(270-int(loc_x+(w/2))) # half of the resolution in X subtracted from the targets X location
#         FOV_x = 62.6 # deg, ratio looked up for field of view for the resolution being used on Camera
#         if x_pixels > 0:
#             theta_x = -(x_pixels*((FOV_x/2)/270)) # assumes to the left is negative
#         else:
#             theta_x = (x_pixels*((FOV_x/2)/270)) # assumes to the right is positive
#         
#         y_pixels = int(250-int(loc_y+(h/2))) # half of the resolution in Y subtracted from the targets Y location
#         FOV_y = 48.8 # deg, ratio looked up for field of view for the resolution being used on Camera
#         if y_pixels > 0:
#             theta_y = (y_pixels*((FOV_y/2)/250)) # assumes up is positive
#         else:
#             theta_y = -(y_pixels*((FOV_y/2)/250)) # assumes down in negative
#     
#         print(theta_x)
#         print(theta_y)
#         
#         endTime = time.time()
#         print("Find location time taken: %s" % (endTime - startTime))
#         
#         #self.camera.stop_preview()
#         return [theta_x,theta_y]
#     
#     def translatePosition(self):
#         #This function converts a screen location to a relative attitude.
#         # Does this above
#         pass
#     
# if __name__ == "__main__":
#         
#     Camera = CameraSingleton()
#     #print(Camera.getPictureString())
#     #Camera.findLocation()
# 
