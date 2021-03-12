import time
import numpy
from threading import Thread
from utilities import log
import user
from user import State
import imu
import numpy as np
import controlLaw
from controlLaw import ControlRoutine
from controlLaw import ypr2quat
from controlLaw import quat2ypr

developMode = True
#This should be set to True when testing on windows. When set to True, the IMU is emulated and
#motor functionality is disabled.

internalWebsocket = True
#This should be set to True when testing on the same machine the script is running on. When set
#to True, the websocket is exposed at the address 'localhost'. Otherwise, static ip is used.

htmlPort = 8009
websocketPort = 8010

if internalWebsocket:
    ip = '127.0.0.1' #Local Machine
else:
    ip = '172.30.0.20' #Static external IP

if developMode:
    log("WARNING: You are in develop mode. Motor and camera functionality is disabled in this mode. To exit develop mode, set 'developMode = False' in main.py")
else:
    import motors

log('Hello world from Prudentia!')

## GUI Servers Setup

# Create a single DataPackage class instance
sharedData = user.SharedDataPackage()

sharedData.state = State.running
sharedData.controlRoutine = ControlRoutine.attitudeInput
sharedData.angularPosition = [0, 0, 0]
sharedData.angularVelocity = [0, 0, 0]
sharedData.qTarget = [0, 0, 0]


log('Setting up GUI server. Accessible on LAN through \"%s:%s\"' % (ip, htmlPort))
log('Data will be exchanged via websockets on LAN through \"%s:%s\"' % (ip, websocketPort))

# Start threads to begin websocket server and html server
# The websocket server is responsible for all data transfer between the GUI and Prudentia
websocketThread = Thread(target=user.startWebsocketServer, args=(sharedData, ip, websocketPort))
websocketThread.start()

# The html server is responsible for delivering html content to the connecting user.
htmlThread = Thread(target=user.startHtmlServer, args=(sharedData, ip, htmlPort))
htmlThread.start()
time.sleep(0.1) #Give the html thread a moment to setup

## IMU Serial Setup

#Create IMU class and open a connection
Imu = imu.ImuSingleton()

log("Opening connection with IMU.")

conn = Imu.openConnection('/dev/ttyUSB0', 115200)
#Start thread to read data asynchronously
if developMode:
    #If develop mode, run IMU emulation (random data)
    log("Develop mode on; emulating IMU. Expect random gyro and accel data.")
    imuReadThread = Thread(target=Imu.emulateImu)

else:
    #Else, connect to the VN-200.
    assert conn is not None #Make sure the port opened correctly.
    imuReadThread = Thread(target=Imu.asyncRead)

imuReadThread.start()
time.sleep(0.1) #Give the imu thread a moment to setup


## Control Law Setup

ControlLaw = controlLaw.ControlLawSingleton()

## Motor Setup

if not developMode:
    Motors = motors.MotorsSingleton()

## Camera Setup

if not developMode:
    import camera
    Camera = camera.CameraSingleton()

## Variable Initialization

lastState = sharedData.state #Store a copy of last state to see mode transitions

loopSpeed = 20 # Hz
allowedTime = 1.0/loopSpeed # Convert Hz to s
times = []

startRuntime = time.time() #Track total runtime for timestamping
loopNumber = 0 #Track the number of total loop iterations.

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

        # Routine change was issued
        if msgJSON["messageType"] == "setRoutine":

            if   msgJSON["routine"] == "stabilize":
                sharedData.controlRoutine = ControlRoutine.stabilize

            elif msgJSON["routine"] == "attitudeInput":
                sharedData.controlRoutine = ControlRoutine.attitudeInput

            elif msgJSON["routine"] == "searchMode":
                sharedData.controlRoutine = ControlRoutine.search

        # Target change was issued
        if msgJSON["messageType"] == "setTarget":

            sharedData.target = msgJSON["target"]

while True:
    loopNumber += 1
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
        sharedData.quaternion = Imu.q.tolist()
        ypr = quat2ypr(Imu.q)
        sharedData.orientation = [ypr.y, ypr.p, ypr.r]

        sharedData.velocity = Imu.w.tolist()
        sharedData.velocityMagnitude = np.linalg.norm(Imu.w)

        sharedData.acceleration = Imu.a.tolist()

        #Set timestamp
        sharedData.timestamp = time.time()

    if sharedData.state == State.standby:
        pass #Do nothing

    elif sharedData.state == State.running:
        #Motors can now be run (No motor code should run outside this statement!)
        if sharedData.controlRoutine == ControlRoutine.stabilize:

            #Run control law with latest IMU data. Stabilize based on current yaw, with zero roll and pitch.
            response = ControlLaw.routineStabilize(Imu.q, Imu.w)

            sharedData.quatTarget = qTarget.tolist()
            sharedData.lqrMode = response.lqrMode.name
            sharedData.qError = response.qError.tolist()
            ypr = quat2ypr(response.qError)
            sharedData.eulerError = [ypr.y, ypr.p, ypr.r]
            sharedData.qErrorAdjusted = response.qErrorAdjusted.tolist()
            sharedData.inertialTorque = response.inertialTorque.tolist()
            sharedData.motorTorque = response.motorTorques.tolist()
            sharedData.motorAccel = response.motorAccel.tolist()

            # Use response to actuate motors
            if not developMode:
                Motors.setAllMotorRpm(response.motorAccel)
                sharedData.currentDC = Motors.currentDC
                sharedData.targetDC = Motors.targetDC

        elif sharedData.controlRoutine == ControlRoutine.attitudeInput:

            #Unpack qTarget from shared data.
            qTarget = controlLaw.ypr2quat(sharedData.qTarget[0], sharedData.qTarget[1], sharedData.qTarget[2])

            #Run control law with latest IMU data and qTarget
            response = ControlLaw.routineAttitudeInput(Imu.q, Imu.w, qTarget)

            sharedData.quatTarget = qTarget.tolist()
            sharedData.lqrMode = response.lqrMode.name
            sharedData.qError = response.qError.tolist()
            ypr = quat2ypr(response.qError)
            sharedData.eulerError = [ypr.y, ypr.p, ypr.r]
            sharedData.qErrorAdjusted = response.qErrorAdjusted.tolist()
            sharedData.inertialTorque = response.inertialTorque.tolist()
            sharedData.motorTorque = response.motorTorques.tolist()
            sharedData.motorAccel = response.motorAccel.tolist()

            # Use response to actuate motors
            if not developMode:
                Motors.setAllMotorRpm(response.motorAccel)
                sharedData.currentDC = Motors.currentDC
                sharedData.targetDC = Motors.targetDC

        elif sharedData.controlRoutine == ControlRoutine.search:
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


    #5 * 0.05 = 0.25
    runtime = time.time() - startRuntime
    targetTime = loopNumber * allowedTime

    if runtime > targetTime:
        log("Loop time exceeded allowance. Loop time: %s. Allowed time: %s" %
            (loopTime, allowedTime))
    else:

        sleepTime = targetTime - runtime
        time.sleep(sleepTime) # Sleep for remaining loop time

        #Report times
        #Instead of reporting 20 times per second, lets report an average over a second.
        if loopNumber % 20 == 0:
            tavg = round(numpy.average(times), 6) # Get average
            tmax = round(numpy.max(times), 6) # Get max
            processingPercent = round(tavg/allowedTime*100, 2) # Get percent of time used
            log("-" * 40)

            log("Loop Times for last second: AVG: [%s ms], MAX: [%s ms], Processing %%: [%s%%]" %
                (tavg, tmax, processingPercent))

            runtime = time.time() - startRuntime
            log("Total Runtime: %s (loop %s), Total Average Frequency: %s Hz" %
                (round(runtime, 4), loopNumber, round(loopNumber/runtime, 4)))

            times = [] # Clear array
