import asyncio
import json
import time
import time

import websockets

import user
import random
from threading import Thread
from user import State
import functools

sharedData = user.DataPackage() #Create a DataPackage class

## Web Server Setup

def startLoop(eventLoop, server):
    eventLoop.run_until_complete(server)
    eventLoop.run_forever()

#Bind sharedData as an extra argument to prudentiaServer
boundServerFunction = functools.partial(user.prudentiaServer, sharedData=sharedData)

#Create a new event loop for the user client
newLoop = asyncio.new_event_loop()

#Create websocket object with our bound function, network options, and the new loop
startServer = websockets.serve(boundServerFunction, '127.0.0.1', 6789, loop = newLoop)

#Start thread to begin server
userThread = Thread(target=startLoop, args=(newLoop, startServer))
userThread.start()

sharedData.state = State.running
print("Starting state: ", sharedData.state," | IMU Data: {", sharedData.imuPosition, "}")
## Main Loop
while True:
    print("State: ", sharedData.state," | IMU Data: {", sharedData.imuPosition, "}")
    time.sleep(0.05)
    #The primary state of Prudentia is an enum defined in user.py in sharedData
    if sharedData.state is State.running:
        #Get and set IMU data
        #For now, random numbers to simulate its moving around
        x = random.randrange(-180, 180)
        y = random.randrange(-180, 180)
        z = random.randrange(-180, 180)
        sharedData.imuPosition = [x, y, z]

    elif sharedData.state is State.standby:
        sharedData.imuPosition[0] += random.randrange(-1, 1)
        sharedData.imuPosition[1] += random.randrange(-1, 1)
        sharedData.imuPosition[2] += random.randrange(-1, 1)
        #Get and set IMU data
        #Motors are off, so x,y,z aren't changing (ideally)

    elif sharedData.state is State.disabled:
        pass
