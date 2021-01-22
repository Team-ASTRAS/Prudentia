from enum import Enum, unique

@unique
class ControlRoutine(Enum):
    standby = 1 #Do nothing [Motors off]
    stabilize = 2 #Stabilize angular position
    realTimeControl = 3 #Change target vector based on RTC input
    attitudeInput = 4 #Set a target vector for Prudentia
    search = 5 #Search routines

class ControlLawSingleton:
    controlRoutine = ControlRoutine.standby

    #Functions named with the format routineName are functions that are called
    #by main.py when the state machine is set to run a particular routine

    def routineStabilize(self):
        pass

    def routineRealTimeControl(self, input):
        pass

    def routineAttitudeInput(self):
        pass

    def routineSearch(self):
        pass

    def __init__(self):
        pass
