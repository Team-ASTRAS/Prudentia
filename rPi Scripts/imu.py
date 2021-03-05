import time
import struct
import numpy as np
from math import sqrt
from time import sleep

import serial
from threading import Thread
from random import uniform
class ImuSingleton:

    #Serial connection attributes
    port = ''
    baudrate = 0
    conn = None
        
    #Imu data fields. These are updated by self.asyncRead() on another thread
    
    q = np.array([0, 0, 0, 1], dtype=float)
    w = np.array([0, 0, 0], dtype=float)
    
    #Packets start with identifier "FA 01 30 00"
    startBytes = bytearray([int('0xfa', 16), int('0x01', 16), int('0x30', 16), int('0x00', 16)])

    def openConnection(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        try:
            self.conn = serial.Serial(self.port, self.baudrate)
            return self.conn
        except:
            print("Failed to open port %s " % str(self.port))

    def literalToFloat(self, literal):
        try:
            returnValue = float(literal)
            return returnValue

        except ValueError:
            try:
                array = literal.split('*')
                returnValue = float(array[0]) * int(array[1], 16)
                return returnValue
            except:
                return None

    #This will update the imu data fields.
    def asyncRead(self):
        output = bytearray()
        
        while True:
            output += self.conn.read(34)
            sleep(0.001)
            if len(output) >= 34:
                
                index = output.find(self.startBytes)
                
                if index != -1 and index + 34 <= len(output):
                    
                    if index != 0:
                        del output[0:index]
                        index = 0
                        
                    if len(output) >= 34:
                        self.processPacket(output[0:34])
                        #print("All data (%s): %s" % (len(output), output))
                        #print("Packet data (%s): %s" % (len(packet), packet))
                        del output[0:34]
                    
    def processPacket(self, packet):
        i = 4
        self.q[0] = struct.unpack('f', packet[i:i+4])[0]
        i+=4
        self.q[1] = struct.unpack('f', packet[i:i+4])[0]
        i+=4
        self.q[2] = struct.unpack('f', packet[i:i+4])[0]
        i+=4
        self.q[3] = struct.unpack('f', packet[i:i+4])[0]
        i+=4
        self.w[0] = struct.unpack('f', packet[i:i+4])[0]
        i+=4
        self.w[1] = struct.unpack('f', packet[i:i+4])[0]
        i+=4
        self.w[2] = struct.unpack('f', packet[i:i+4])[0]
        
    def emulateImu(self):
        
        #Produces a unit quaternion with uniform random rotation
        x = y = z  =  u = v = w  =  s = float(0)
        while True:
            
            #Produce a random x and y until z > 1 if z = x**2 + y**2
            while True:
                x = uniform(-1, 1)
                y = uniform(-1, 1)
                z = x**2 + y**2
                if(z < 1):
                    break
                
            #Produce a random u and v until w > 1 if w = u**2 + v**2
            while True:
                u = uniform(-1, 1)
                v = uniform(-1, 1)
                w = u**2 + v**2
                if(w < 1):
                    break
                
            s = sqrt((1-z) / w)
                
            self.q = np.array([x, y, s*u, s*v])
                    
            self.w = np.array([uniform(-0.1,0.1), uniform(-0.1,0.1), uniform(-0.1,0.1)])
                    
            sleep(0.005)
            
            #print("Q: %s" % q)
            #print("W: %s" % w)
            
if __name__ == "__main__":
    Imu = ImuSingleton()
    conn = Imu.openConnection('/dev/ttyUSB0', 115200) # Raspi USB
    #conn = Imu.openConnection('com13', 115200) # Windows USB
    assert conn is not None
    serialThread = Thread(target=Imu.asyncRead)
    serialThread.start()
    while True:
        time.sleep(0.05)
        print("Q:%s, W:%s" % (Imu.q, Imu.w))
