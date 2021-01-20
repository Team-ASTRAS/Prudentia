import asyncio
import signal
import time, numpy, controlLaw
from threading import Thread
import websockets
from enum import Enum, unique
from utilities import log
import user

from user import State

@unique
class ControlRoutine(Enum):
    standby = 1 #Do nothing [Motors off]
    stabilize = 2 #Stabilize angular position
    realTimeControl = 3 #Change target vector based on RTC input
    attitudeInput = 4 #Set a target vector for Prudentia
    search = 5 #Search routines

controlRoutine = ControlRoutine.standby

## Web Server Setup
# Create a single DataPackage class instance
sharedData = user.DataPackage()

# Start thread to begin server
userThread = Thread(target=user.startServer, args=(sharedData,))
userThread.start()

## Initialization

sharedData.state = State.running
sharedData.angularPosition = [0, 0, 0]
sharedData.angularVelocity = 0
sharedData.target = [0, 0, 0]

lastState = sharedData.state #Store a copy of last state to see mode transitions

loopSpeed = 20 # Hz
times = []

log('Hello world from Prudentia!')
log('Setting up..')

ControlLaw = controlLaw.ControlLaw()

def processCommands():
    while not sharedData.commandQueue.empty():
        msgJSON = sharedData.commandQueue.get()
        if msgJSON["messageType"] == "setState":
            if msgJSON["state"] == "disabled":
                sharedData.state = State.shutdown
            elif msgJSON["state"] == "standby":
                sharedData.state = State.standby
            elif msgJSON["state"] == "running":
                sharedData.state = State.running

while True:
    loopStart = time.time()

    #Check sharedData.commandQueue for any incoming commands
    processCommands()

    # If state changes, update and log
    if sharedData.state != lastState:
        log("Mode changed from %s to %s." % (lastState, sharedData.state))
        lastState = sharedData.state

    # Switch behavior depending on state
    if sharedData.state == State.shutdown:
        #Stop server (defined in startServer in user.py)
        sharedData.stopServer.set_result(None)
        break #If disabled, end program

    elif sharedData.state == State.standby:
        pass #Do nothing

    elif sharedData.state == State.running:
        # We are now in control, get IMU data
        # current attitude from imu.py

        sharedData.angularPosition[0] += 1

        if controlRoutine == ControlRoutine.standby:
            # Do nothing
            pass

        elif controlRoutine == ControlRoutine.stabilize:
            # Stabilize
            # Set ControlLaw Data
            response = ControlLaw.routineStabilize()
            # Use response to actuate motors

        elif controlRoutine == ControlRoutine.realTimeControl:
            # Real time control
            # Set ControlLaw Data
            response = ControlLaw.routineRealTimeControl()
            # Use response to actuate motors

        elif controlRoutine == ControlRoutine.attitudeInput:
            # Attitude input
            # Set ControlLaw Data
            response = ControlLaw.routineAttitudeInput()
            # Use response to actuate motors

        elif controlRoutine == ControlRoutine.search:
            # Search
            # Set ControlLaw Data
            response = ControlLaw.routineSearch()
            # Use response to actuate motors

        else:
            log("Error: controlRoutine not found")
            pass


    ## Timing

    loopEnd = time.time()
    loopTime = loopEnd - loopStart
    times.append(loopTime)  # Record time

    allowedTime = 1.0/loopSpeed # Convert Hz to s

    if loopTime > allowedTime:
        log("Loop time exceeded allowance. Loop time: %s. Allowed time: %s" %
            (loopTime, allowedTime))
    else:
        sleepTime = allowedTime - loopTime
        time.sleep(sleepTime) # Sleep for remaining loop time


    #Report times
    #Instead of reporting 20 times per second, lets report an average over a second.
    if len(times) >= loopSpeed:
        tavg = round(numpy.average(times), 4) # Get average
        tmax = round(numpy.max(times), 4) # Get max
        processingPercent = round(tavg/allowedTime*100, 1) # Get percent of time used

        log("Loop Times for last second: AVG: [%s ms], MAX: [%s ms], Processing %%: [%s%%]" %
            (tavg, tmax, processingPercent))

        times = [] # Clear array

