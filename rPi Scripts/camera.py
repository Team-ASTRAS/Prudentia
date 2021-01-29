

class CameraSingleton:

    def __init__(self):
        #Camera setup.
        #Set resolution, other settings
        
        import picamera
        from time import sleep
        from picamera import PiCamera
        camera = PiCamera()
        camera.resolution = (1024,768)
        camera.framerate = 15

    def getPicture(self):
        #This now gets a picture
        #Return bitmap/whatever format. Preferably a fast format if we can choose

        for i in range(5) 
            camera.start_preview()
            camera.capture('/home/pi/Desktop/image%s.bmp' % i)
            camera.stop_preview()
        pass

    def findLocation(self, picture):
        #Return screen position based on target in picture
        #Or, return None if no target is found
        #This image processing algorithm is going to have to be fast. Hopefully IR contrast helps enough
        pass

    def translatePosition(self, location):
        #This function converts a screen location to a relative attitude.
        pass