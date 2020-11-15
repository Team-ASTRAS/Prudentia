import numpy as np

# Modes
# shutdown - no control
# standby - orient at [0, 0, 0] and await commands
# trackattitude - track attitude at RPYtarget
# searchtarget - search for target and lock on

class User:

    def __init__(self):
        pass

    mode = "trackattitude"
    target = [30, 10, 120]

    def getLatestCommand(self):
        return self.mode

    def getLatestTarget(self):
        return self.target

    def sendData(self, data):
        #send data
        #print(data)
        pass




