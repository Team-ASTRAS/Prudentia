#!/usr/bin/env python

import asyncio
import json
import logging
import random
import time

import websockets
import functools
from threading import Lock
from main import State

dataLock = Lock()
# Usage:    with dataLock:
#               sharedData.variable = x



class DataPackage:

    state = State(State.standby)
    imuPosition = [0, 0, 0]
    angularVelocity = 0
    target = [0, 0, 0]

    def getDataJson(self):
        dataObject = {  "state" : self.state.name,
                        "ImuPositionEuler" : self.imuPosition,
                        "angularVelocity" : self.angularVelocity,
                        "target" : self.target}
        return json.dumps(dataObject)

async def prudentiaServer(websocket, path, sharedData):
    #We have access to sharedData from this function
    print("Connected to user")

    while True:
        try:
            async for message in websocket:
                msgJSON = json.loads(message)

                if msgJSON["messageType"] == "getData":
                    with dataLock:
                        print("Sending server data: ", sharedData.getDataJson())
                        await websocket.send(sharedData.getDataJson())

                if msgJSON["messageType"] == "setState":
                    with dataLock:
                        if msgJSON["state"] == "disabled":
                            sharedData.state = State.disabled
                        elif msgJSON["state"] == "standby":
                            sharedData.state = State.standby
                        elif msgJSON["state"] == "running":
                            sharedData.state = State.running
        finally:
            pass
            #Consider sending disconnect message?
