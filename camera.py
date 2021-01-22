

class CameraSingleton:

    def __init__(self):
        #Camera setup.
        #Set resolution, other settings
        pass

    def getPicture(self):
        #This now gets a picture

        #Return bitmap/whatever format. Preferably a fast format if we can choose
        pass

    def findLocation(self, picture):
        #Return screen position based on target in picture
        #Or, return None if no target is found
        #This image processing algorithm is going to have to be fast. Hopefully IR contrast helps enough
        pass

    def translatePosition(self, location):
        #This function converts a screen location to a relative attitude.
        pass