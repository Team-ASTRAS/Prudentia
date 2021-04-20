
import picamera
import time

camera = picamera.PiCamera()

resolution = 500

camera.resolution = (resolution, resolution) # if this changes then the FOV math below needs to be updated
camera.framerate = 24

camera.start_preview()
time.sleep(30)
camera.stop_preview()