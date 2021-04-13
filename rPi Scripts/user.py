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
import csv
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

    recordingData = ""
    LogType = ""

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

    duty = [0, 0, 0, 0]
    currentRpm = [0, 0, 0, 0]
    targetRpm = [0, 0, 0, 0]

    stopServer = None

    image = ""

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

                        "duty" : self.duty,                     #Current Duty Cycle of the motors
                        "currentRpm" : self.currentRpm,         #Current RPM array of motors
                        "targetRpm" : self.targetRpm,           #Target RPM array of motors

                        "image" : self.image,                   #Camera images

                        "timestamp" : self.timestamp
                        }

        return json.dumps(dataObject)

    def deleteData(self):
        filename = "TempData.csv"
        temp = open(filename, 'w')
        temp.truncate(0)

    def sendCsv(self, filenameToSave):
        localFile = "TempData.csv"
        with open(localFile) as csvf:
            reader = csv.reader(csvf)
            stringyBoi = ""
            for row in reader:
                stringyBoi += str(",".join(row)) + "\n"
            csvJson = {"csvFile" : stringyBoi, "filename" : filenameToSave}
            return json.dumps(csvJson)

    def RecordData(self):
        filename = "TempData.csv"
        if os.stat(filename).st_size == 0:
            with open(filename, 'a', newline='', encoding='utf-8') as temp:
                headers = ["State","Routine","Timestamp","Yaw","Pitch","Roll","q1",
                            "q2","q3","q4","Vel_Yaw","Vel_Pitch","Vel_Roll",
                            "Vel_Mag","Accel_Yaw","Accel_Pitch","Accel_Roll",
                            "Target_Yaw","Target_Pitch","Target_Roll", "q1_Error",
                            "q2_Error","q3_Error","q4_Error","Yaw_Error",
                            "Pitch_Error","Roll_Error","Req_InertTorque_Yaw",
                            "Req_InertTorque_Pitch","Req_InertTorque_Roll",
                            "Req_Torque_M1","Req_Torque_M2","Req_Torque_M3",
                            "Req_Torque_M4","Accel_M1","Accel_M2","Accel_M3",
                            "Accel_M4","Duty_M1","Duty_M2","Duty_M3","Duty_M4",
                            "RPM_M1","RPM_M2","RPM_M3","RPM_M4","TargetRPM_M1",
                            "TargetRPM_M2","TargetRPM_M3","TargetRPM_M4"]
                filename_writer = csv.writer(temp)
                filename_writer.writerow(headers)

        with open(filename, 'a', newline='', encoding='utf-8') as temp:
            filename_writer = csv.writer(temp, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
            filename_writer.writerow([self.state.name, self.controlRoutine.name,
                                    self.timestamp, self.orientation[0],
                                    self.orientation[1], self.orientation[2],
                                    self.quaternion[0], self.quaternion [1],
                                    self.quaternion [2], self.quaternion [3],
                                    self.velocity [0], self.velocity [1],
                                    self.velocity [2], self.velocityMagnitude,
                                    self.acceleration [0], self.acceleration [1],
                                    self.acceleration [2], self.target [0],
                                    self.target [1], self.target [2],
                                    self.qError [0], self.qError [1],
                                    self.qError [2], self.qError [3],
                                    self.eulerError [0], self.eulerError [1],
                                    self.eulerError [2], self.inertialTorque [0],
                                    self.inertialTorque [1], self.inertialTorque [2],
                                    self.motorTorque [0], self.motorTorque [1],
                                    self.motorTorque [2], self.motorTorque [3],
                                    self.motorAccel [0], self.motorAccel [1],
                                    self.motorAccel [2], self.motorAccel [3],
                                    self.duty [0], self.duty [1], self.duty [2],
                                    self.duty [3], self.currentRpm [0],
                                    self.currentRpm [1], self.currentRpm [2],
                                    self.currentRpm [3], self.targetRpm [0],
                                    self.targetRpm [1], self.targetRpm [2],
                                    self.targetRpm [3]])

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

            elif msgJSON["messageType"] == "downloadData":
                await websocket.send(sharedData.sendCsv(msgJSON["filename"]))

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


if __name__ == "__main__":
    
    sharedData = SharedDataPackage()
    sharedData.state = State.running
    from controlLaw import ControlRoutine
    sharedData.controlRoutine = ControlRoutine.attitudeInput
    sharedData.angularPosition = [0, 0, 0]
    sharedData.angularVelocity = [0, 0, 0]
    sharedData.target = [0, 0, 0]

    for i in range(10):
       sharedData.orientation[0] = i
       filename = "TempData.csv"
       sharedData.RecordData()