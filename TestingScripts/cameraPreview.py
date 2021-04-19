
import picamera
import time

camera = picamera.PiCamera()

resolution = 500

camera.resolution = (resolution, resolution) # if this changes then the FOV math below needs to be updated
camera.framerate = 24

camera.start_preview()
time.sleep(3)
camera.capture('/home/pi/Desktop/Prudentia/rPi Scripts/images/image0.bmp')
time.sleep(3)
camera.capture('/home/pi/Desktop/Prudentia/rPi Scripts/images/image1.bmp')
time.sleep(3)
camera.capture('/home/pi/Desktop/Prudentia/rPi Scripts/images/image2.bmp')
time.sleep(3)
camera.capture('/home/pi/Desktop/Prudentia/rPi Scripts/images/image3.bmp')
time.sleep(3)
camera.capture('/home/pi/Desktop/Prudentia/rPi Scripts/images/image4.bmp')
camera.stop_preview()