import time, numpy, controlLaw
import user
from utilities import log

mode = "shutdown"
target = [0, 0, 0]

loopSpeed = 20 # Hz
times = []

log('Hello world!')

log('Setting up..')
User = user.User() # Set up new User class instance named User
ControlLaw = controlLaw.ControlLaw()

while True:

    loopStart = time.time()

    # Get latest mode
    latestMode = User.getLatestCommand()

    # If mode changes, update and log
    if mode != latestMode:
        log("Mode changed from %s to %s." % (mode, latestMode))
        mode = latestMode

    # Switch behavior depending on mode

    if mode == "shutdown":
        pass
    else:
        # We are now in control, get IMU data
        # current attitude from imu.py

        if mode == "standby":
            # Set target to 0 0 0
            target = [0, 0, 0]

        elif mode == "trackattitude":
            # Get latest target attitude from user.py
            target = User.getLatestTarget

        elif mode == "searchtarget":
            # Custom search algorithm:
            # Try for a target from camera.py
            # If target, set attitude.
            # If no target, progressively yaw sweep at 0 pitch.
            pass

        # We now have IMU data and target data
        # Pass information to control law
        # Call control law, receive torque and other data
        ControlLaw.go()

        # We now have desired torques
        # Call motors.py to calculate the signals and return data

        # Control law has been calculated and motors actuated.
        # Now, return data to user interface.

        User.sendData("Current attitude, target attitude, other stuff")

        #time.sleep(0.001)

        # Timing math

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

