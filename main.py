import asyncio
import signal
import time, numpy, controlLaw
from threading import Thread
import websockets
from utilities import log
import user

from user import State
from controlLaw import ControlRoutine
import imu

log('Hello world from Prudentia!')
log('Setting up..')

## Web Server Setup

# Create a single DataPackage class instance
sharedData = user.DataPackage()

# Start thread to begin server
userThread = Thread(target=user.startServer, args=(sharedData,))
userThread.start()

log("Websocket server started")


## IMU Serial Setup

#Create IMU class and open a connection
Imu = imu.ImuSingleton()
conn = Imu.openConnection('com4', 9600)
assert conn is not None #Make sure the port opened correctly

#Start thread to read data asynchronously
serialThread = Thread(target=Imu.asyncRead)
serialThread.start()


## Control Law Setup

ControlLaw = controlLaw.ControlLawSingleton()

## Motor Setup
import motors
Motors = motors.MotorsSingleton()

## Camera Setup

import camera
Camera = camera.CameraSingleton()

## Variable Initialization

sharedData.state = State.running
sharedData.angularPosition = [0, 0, 0]
sharedData.angularVelocity = [0, 0, 0]
sharedData.target = [0, 0, 0]

lastState = sharedData.state #Store a copy of last state to see mode transitions

loopSpeed = 20 # Hz
times = []



def processCommands():
    while not sharedData.commandQueue.empty():
        msgJSON = sharedData.commandQueue.get()

        # State change was issued
        if msgJSON["messageType"] == "setState":

            if   msgJSON["state"] == "shutdown":
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
        #TODO if changing from running, make sure we turn motors off
    controlLawData = {}
    # Switch behavior depending on state
    if sharedData.state == State.shutdown:
        #Stop server (defined in startServer in user.py)
        sharedData.stopServer.set_result(None)
        break #If disabled, end program

    elif sharedData.state == State.standby:
        pass #Do nothing

    elif sharedData.state == State.running:
        #Motors can now be run (No motor code should run outside this statement!)

        if ControlLaw.controlRoutine == ControlRoutine.standby:
            # Do nothing
            pass

        elif ControlLaw.controlRoutine == ControlRoutine.stabilize:
            # Stabilize
            # Set ControlLaw Data
            response = ControlLaw.routineStabilize()
            # Use response to actuate motors

        elif ControlLaw.controlRoutine == ControlRoutine.realTimeControl:
            # Real time control
            # Set ControlLaw Data
            response = ControlLaw.routineRealTimeControl(controlLawData)
            # Use response to actuate motors

        elif ControlLaw.controlRoutine == ControlRoutine.attitudeInput:
            # Attitude input
            # Set ControlLaw Data
            response = ControlLaw.routineAttitudeInput()
            # Use response to actuate motors

        elif ControlLaw.controlRoutine == ControlRoutine.search:
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

