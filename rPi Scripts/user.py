#!/usr/bin/env python
import http.server
import socketserver
import websockets
import asyncio
import json
import queue
import functools
import os
import time
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
    controlRoutine = ""

    quaternion = [0, 0, 0, 0]
    orientation = [0, 0, 0]
    velocity = [0, 0, 0]
    velocityMagnitude = 0
    acceleration = [0, 0, 0]
    
    quatTarget = [0, 0, 0, 0]
    target = [0, 0, 0]
    lqrMode = ""
    qError = [0, 0, 0, 0]
    eulerError = [0, 0, 0]
    qErrorAdjusted = [0, 0, 0, 0]
    inertialTorque = [0, 0, 0]
    motorTorque = [0, 0, 0, 0]
    motorAccel = [0, 0, 0, 0]

    currentDC = [0, 0, 0, 0]
    targetDC = [0, 0, 0, 0]

    stopServer = None

    timestamp = time.time()

    def getDataJson(self):
        dataObject = {  "state" : self.state.name,              #Current state (running, standby, etc)
                        "routine" : self.controlRoutine.name,   #Control Routine (stabilize, AI, search)

                        "quaternion" : self.quaternion,         #qhat q4
                        "orientation" : self.orientation,       #Euler Angle of quaternion
                        "velocity" : self.velocity,             #w in rad/s (IMU Gyros)
                        "velocityMag" : self.velocityMagnitude, #mag of w
                        "acceleration" : self.acceleration,     #a in m/s^2 (IMU Accels)

                        "target" : self.target,                 #User defined target in YPR
                        "lqrMode" : self.lqrMode,               #Current control law mode (nominal, yaw sweep, etc)
                        "qError" : self.qError,                 #Error between target and quaternion
                        "eulerError" : self.eulerError,         #Euler Angle version of qError
                        "qErrorAdjusted" : self.qErrorAdjusted, #Error after corrective adjustment
                        "inertialTorque" : self.inertialTorque, #3 axis inertial torque output from control law
                        "motorTorque" : self.motorTorque,       #Torque requested for each motor
                        "motorAccel" : self.motorAccel,         #Acceleration requested for each motor
                        
                        "currentDC" : self.currentDC,           #Current Duty Cycle of the motors
                        "targetDC" : self.targetDC,             #Target Duty Cycle of the motors

                        "timestamp" : self.timestamp
                        }             

        return json.dumps(dataObject)

    def saveJson(self):
        with open('data.txt', 'w') as outfile:
            json.dump(data, outfile)

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
# Redirect http GETs to our html content
class MyHttpRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.path = 'Homepage.html'
        return http.server.SimpleHTTPRequestHandler.do_GET(self)

# Run the html server
def startHtmlServer(sharedDataRef, ip, port):

    guiDir = os.path.join(os.path.dirname(__file__), 'GUI Files')
    os.chdir(guiDir)
    log("Loading Gui Files from directory: \"" + os.getcwd() + "\"") 

    handle = MyHttpRequestHandler
    #socketserver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
    sharedDataRef.htmlServer = socketserver.TCPServer((ip, port), handle)

    sharedDataRef.htmlServer.serve_forever()
