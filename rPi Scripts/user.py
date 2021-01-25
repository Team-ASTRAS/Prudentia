#!/usr/bin/env python
import http.server
import socketserver
import websockets
import asyncio
import json
import queue
import functools
from enum import Enum, unique
from utilities import log


@unique
class State(Enum):
    standby = 1 #Loop is running and server is active, waiting for input [Motors off]
    running = 2 #Running routine specified by user (goto coordinate, tracking, calibrate, etc) [Motors on
    shutdown = 3 #Program has crashed and cannot continue [Motors off]

#This class defines shared data between the server thread and main thread
class SharedDataPackage:

    #This queue is filled by the user GUI sending commands
    #This queue is processed at the beginning of each loop in main
    commandQueue = queue.Queue()

    state = State.standby
    angularPosition = [0, 0, 0]
    angularVelocity = [0, 0, 0]
    angularVelocityMagnitude = 0
    target = [0, 0, 0]

    stopServer = None


    def getDataJson(self):
        dataObject = {  "state" : self.state.name,
                        "angularPosition" : self.angularPosition,
                        "angularVelocity" : self.angularVelocity,
                        "target" : self.target}
        return json.dumps(dataObject)

## Websocket Server

#Start point of thread function
def startWebsocketServer(sharedDataRef, ip, port):

    # Create a new event loop for the user client
    eventLoop = asyncio.new_event_loop()

    # Create signal to terminate server at a later time
    sharedDataRef.stopWebsocketServer = eventLoop.create_future()

    # Bind sharedData as an extra argument to prudentiaServer
    boundServerFcn = functools.partial(prudentiaServer, sharedData=sharedDataRef)

    eventLoop.run_until_complete(websocketServerHelper(boundServerFcn, eventLoop, sharedDataRef.stopWebsocketServer, ip, port))

#This function helps us shut down the server later
async def websocketServerHelper(boundServerFcn, eventLoop, stop, ip, port):
    # Create websocket object with our bound function, network options, and the new loop
    log("Websocket server started on port %d" % port)
    async with websockets.serve(boundServerFcn, ip, port, loop=eventLoop):
        await stop # when the stop object .set_result() method is called, server shuts down
        log("Websocket server thread terminated.")

#This function is run whenever a client connects to our websocket
async def prudentiaServer(websocket, path, sharedData):
    #We have access to sharedData from this function
    log("Connected to server.")
    try:
        async for message in websocket:
            msgJSON = json.loads(message)

            if msgJSON["messageType"] == "getData":
                log("Sending data to client.")
                await websocket.send(sharedData.getDataJson())

            else:
                sharedData.commandQueue.put(msgJSON)

    except websockets.WebSocketException as e:
        log("Websocket experienced an error: %s %s" % ( e, "Closing"))
        await websocket.close()

    log("User disconnected")
    await asyncio.sleep(1)


## HTML Server
import os
# Redirect http GETs to our html content
class MyHttpRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            log("CWD:" + os.getcwd())
            self.path = 'Homepage.html'
            log("Set Path:" + os.path.realpath(self.path))
        return http.server.SimpleHTTPRequestHandler.do_GET(self)

# Run the html server
def startHtmlServer(sharedDataRef, ip, port):

    guiDir = os.path.join(os.path.dirname(__file__), 'Gui Files')
    os.chdir(guiDir)

    handle = MyHttpRequestHandler
    sharedDataRef.htmlServer = socketserver.TCPServer((ip, port), handle)

    sharedDataRef.htmlServer.serve_forever()
