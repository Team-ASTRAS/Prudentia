import time
import numpy
from threading import Thread
from utilities import log
import user
from user import State
import camera
import imu
#import motors
import controlLaw
from controlLaw import ControlRoutine

log('Hello world from Prudentia!')
log('Setting up..')

## GUI Servers Setup

# Create a single DataPackage class instance
sharedData = user.SharedDataPackage()

ip = '127.0.0.1' #Local Machine

# Start threads to begin websocket server and html server
# The websocket server is responsible for all data transfer between the GUI and Prudentia
websocketThread = Thread(target=user.startWebsocketServer, args=(sharedData, ip, 8010))
websocketThread.start()

# The html server is responsible for delivering html content to the connecting user.
htmlThread = Thread(target=user.startHtmlServer, args=(sharedData, ip, 8009))
htmlThread.start()
time.sleep(1) #Give the html thread a moment to setup

## IMU Serial Setup

#Create IMU class and open a connection
Imu = imu.ImuSingleton()
conn = Imu.openConnection('com4', 9600)
#assert conn is not None #Make sure the port opened correctly

#Start thread to read data asynchronously
#imuReadThread = Thread(target=Imu.asyncRead)
imuReadThread = Thread(target=Imu.emulateImu)
imuReadThread.start()
time.sleep(1) #Give the imu thread a moment to setup


## Control Law Setup

ControlLaw = controlLaw.ControlLawSingleton()

## Motor Setup

#Motors = motors.MotorsSingleton()

## Camera Setup

Camera = camera.CameraSingleton()

## Variable Initialization

sharedData.state = State.running
sharedData.controlRoutine = ControlRoutine.stabilize
sharedData.angularPosition = [0, 0, 0]
sharedData.angularVelocity = [0, 0, 0]
sharedData.qTarget = [0, 0, 0]

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

    # Switch behavior depending on state
    if sharedData.state == State.shutdown:
        #Stop websocket server (stop object defined in startWebsocketServer in user.py)
        sharedData.stopWebsocketServer.set_result(None)
        log("Websocket server shut down.")

        #Stop html server (server reference defined in startHtmlServer in user.py)
        sharedData.htmlServer.shutdown()
        sharedData.htmlServer.server_close()
        log("HTML Server shut down.")

        log("Main thread shutting down.")
        break #If disabled, end program

    if (sharedData.state and State.standby) or (sharedData.state and State.running):
        #Set IMU data
        pass#sharedData.angularPosition = Imu.position


    if sharedData.state == State.standby:
        pass #Do nothing

    elif sharedData.state == State.running:
        #Motors can now be run (No motor code should run outside this statement!)
        if sharedData.controlRoutine == ControlRoutine.stabilize:
            
            #Run control law with latest IMU data. Stabilize based on current yaw, with zero roll and pitch.
            response = ControlLaw.routineStabilize(Imu.q, Imu.w)

            # Use response to actuate motors
            #Motors.setAllMotorRpm(response.motorAccel)

        elif sharedData.controlRoutine == ControlRoutine.attitudeInput:
            
            #Unpack qTarget from shared data.
            qTarget = controlLaw.ypr2quat(sharedData.qTarget[0], sharedData.qTarget[1], sharedData.qTarget[2])
            
            #Run control law with latest IMU data and qTarget
            response = ControlLaw.routineAttitudeInput(Imu.q, Imu.w, qTarget)
            
            # Use response to actuate motors
            #Motors.setAllMotorRpm(response.motorAccel)
            
        elif sharedData.controlRoutine == ControlRoutine.search:
            # Search
            # Set ControlLaw Data
            response = ControlLaw.routineSearch()
            # Use response to actuate motors

        else:
            log("Error: controlRoutine not found")
            pass


    ## Timing
    #TODO This loop system has a bit of overhead. Improve it. See 'logging' python module

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

