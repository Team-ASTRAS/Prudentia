from enum import Enum, unique

@unique
class ControlRoutine(Enum):
    stabilize = 1 #Stabilize angular position
    realTimeControl = 2 #Change target vector based on RTC input
    attitudeInput = 3 #Set a target vector for Prudentia
    search = 4 #Search routines

class ControlLawSingleton:

    controlRoutine = ControlRoutine.stabilize

    #Functions named with the format routineName are functions that are called
    #by main.py when the state machine is set to run a particular routine

    def routineStabilize(self):
        pass

    def routineRealTimeControl(self):
        pass

    def routineAttitudeInput(self):
        pass

    def routineSearch(self):
        pass

    def __init__(self):
        pass
