import asyncio
import functools
import time, numpy, controlLaw
from threading import Thread
import websockets
import user
from enum import Enum, unique
from utilities import log

@unique
class State(Enum):
    standby = 1 #Loop is running and server is active, waiting for input [Motors off]
    running = 2 #Running routine specified by user (goto coordinate, tracking, calibrate, etc) [Motors on
    disabled = 3 #Program has crashed and cannot continue [Motors off]

@unique
class ControlRoutine(Enum):
    standby = 1
    stabilize = 2
    realTimeControl = 3
    attitudeInput = 4
    search = 5

controlRoutine = ControlRoutine.standby

## Web Server Setup

sharedData = user.DataPackage() #Create a DataPackage class

def startLoop(eventLoop, server):
    eventLoop.run_until_complete(server)
    eventLoop.run_forever()

# Bind sharedData as an extra argument to prudentiaServer
boundServerFunction = functools.partial(user.prudentiaServer, sharedData=sharedData)

# Create a new event loop for the user client
newLoop = asyncio.new_event_loop()

# Create websocket object with our bound function, network options, and the new loop
startServer = websockets.serve(boundServerFunction, '127.0.0.1', 6789, loop=newLoop)

# Start thread to begin server
userThread = Thread(target=startLoop, args=(newLoop, startServer))
userThread.start()


## Initialization
running = True

sharedData.state = State.running
sharedData.imuPosition = [0, 0, 0]
sharedData.angularVelocity = 0
sharedData.target = [0, 0, 0]

lastState = sharedData.state #Store a copy of last state to see mode transitions

loopSpeed = 20 # Hz
times = []

log('Hello world from Prudentia!')
log('Setting up..')

ControlLaw = controlLaw.ControlLaw()

while running:
    loopStart = time.time()

    # If state changes, update and log
    if sharedData.state != lastState:
        log("Mode changed from %s to %s." % (lastState, sharedData.state))
        lastState = sharedData.state

    # Switch behavior depending on state
    if sharedData.state == State.disabled:
        break #If disabled, end program
    elif sharedData.state == State.standby:
        pass #Do nothing
    elif sharedData.state == State.running:
        # We are now in control, get IMU data
        # current attitude from imu.py

        if controlRoutine == ControlRoutine.standby:
            # Do nothing
            pass

        elif controlRoutine == ControlRoutine.stabilize:
            # Stabilize
            # Set ControlLaw Data
            response = ControlLaw.routineStabilize()
            # Use response to actuate motors

        elif controlRoutine == ControlRoutine.realTimeControl:
            # Stabilize
            # Set ControlLaw Data
            response = ControlLaw.routineRealTimeControl()
            # Use response to actuate motors

        elif controlRoutine == ControlRoutine.attitudeInput:
            # Stabilize
            # Set ControlLaw Data
            response = ControlLaw.routineAttitudeInput()
            # Use response to actuate motors

        elif controlRoutine == ControlRoutine.search:
            # Stabilize
            # Set ControlLaw Data
            response = ControlLaw.routineSearch()
            # Use response to actuate motors

        else:
            log("Error: controlRoutine not found")
            pass

        #time.sleep(0.001)

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

