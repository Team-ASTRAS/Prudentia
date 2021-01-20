#!/usr/bin/env python
import asyncio
import json
import queue
import functools
from enum import Enum, unique

import websockets

@unique
class State(Enum):
    standby = 1 #Loop is running and server is active, waiting for input [Motors off]
    running = 2 #Running routine specified by user (goto coordinate, tracking, calibrate, etc) [Motors on
    shutdown = 3 #Program has crashed and cannot continue [Motors off]

#This class defines shared data between the server thread and main thread
class DataPackage:

    #This queue is filled by the user GUI sending commands
    #This queue is processed at the beginning of each loop in main
    commandQueue = queue.Queue()

    state = State.standby
    angularPosition = [0, 0, 0]
    angularVelocity = 0
    target = [0, 0, 0]


    def getDataJson(self):
        dataObject = {  "state" : self.state.name,
                        "angularPosition" : self.angularPosition,
                        "angularVelocity" : self.angularVelocity,
                        "target" : self.target}
        return json.dumps(dataObject)

#Start point of thread function
def startServer(sharedDataRef):

    # Create a new event loop for the user client
    eventLoop = asyncio.new_event_loop()

    # Create signal to terminate server at a later time
    sharedDataRef.stopServer = eventLoop.create_future()

    # Bind sharedData as an extra argument to prudentiaServer
    boundServerFcn = functools.partial(prudentiaServer, sharedData=sharedDataRef)

    eventLoop.run_until_complete(serverHelper(boundServerFcn, eventLoop, sharedDataRef.stopServer))

#This function helps us shut down the server later
async def serverHelper(boundServerFcn, eventLoop, stop):
    # Create websocket object with our bound function, network options, and the new loop
    async with websockets.serve(boundServerFcn, '127.0.0.1', 6789, loop=eventLoop):
        await stop # when the stop object .set_result() method is called, server shuts down

#This function is run whenever a client connects to our websocket
async def prudentiaServer(websocket, path, sharedData):
    #We have access to sharedData from this function
    print("Connected to user")
    try:
        async for message in websocket:
            msgJSON = json.loads(message)

            if msgJSON["messageType"] == "getData":
                print("Sending data to client")
                await websocket.send(sharedData.getDataJson())

            else:
                sharedData.commandQueue.put(msgJSON)
    except:
        print("Websocket experienced an error: Closing")
        await websocket.close()

    print("User disconnected")
    await asyncio.sleep(1)