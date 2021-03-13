import time
import struct
import numpy as np
from math import sqrt, cos, sin
from time import sleep

import serial
from controlLaw import eulerAxis2quat, quatMultiply, ypr2str, quat2ypr, w2str
from threading import Thread
from random import uniform

class ImuSingleton:

    #Serial connection attributes
    port = ''
    baudrate = 0
    conn = None
    packetLength = 30    
    updateFrequency = 50 #Hz
        
    #Imu data fields. These are updated by self.asyncRead() on another thread
    q = np.array([0, 0, 0, 1], dtype=float)
    w = np.array([0, 0, 0], dtype=float)
    a = np.array([0, 0, 0], dtype=float)
    
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
            output += self.conn.read(30)
            sleep(0.001)
            if len(output) >= self.packetLength:
                
                index = output.find(self.startBytes)
                
                if index != -1 and index + self.packetLength <= len(output):
                    
                    if index != 0:
                        del output[0:index]
                        index = 0
                        
                    if len(output) >= self.packetLength:
                        self.processPacket(output[0:self.packetLength])
                        #print("All data (%s): %s" % (len(output), output))
                        #print("Packet data (%s): %s" % (len(packet), packet))
                        del output[0:self.packetLength]
                    
    def processPacket(self, packet):
        i = 4
        self.a[1] = struct.unpack('f', packet[i:i+4])[0]
        i+=4
        self.a[2] = struct.unpack('f', packet[i:i+4])[0]
        i+=4
        self.a[3] = struct.unpack('f', packet[i:i+4])[0]

        i+=4
        self.w[0] = struct.unpack('f', packet[i:i+4])[0]
        i+=4
        self.w[1] = struct.unpack('f', packet[i:i+4])[0]
        i+=4
        self.w[2] = struct.unpack('f', packet[i:i+4])[0]

        self.propogateQuaternion()

    def propogateQuaternion(self):
        w = np.deg2rad(self.w) / self.updateFrequency
        wMag = np.linalg.norm(w)
        wAxis = w / wMag

        if(wMag < 0.01):
            pass #TODO Recalibrate P, R

        deltaQ = eulerAxis2quat(wAxis, wMag)
        self.q = quatMultiply(self.q, deltaQ)
    
    def setCalibration(self):
        print("Calibrating! Accels: %s" % self.a)

    def emulateImu(self):
        while True:
            sleep(0.02)

            self.w = 100 * np.array([uniform(-0.1,0.1), uniform(-0.1,0.1), uniform(-0.1,0.1)])
                    
            self.propogateQuaternion()
            print("Propogation Data: %s      w: %s" % (ypr2str(quat2ypr(Imu.q)), w2str(self.w)))
                   
    def getRandomQuat(self):
        
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
                    
            sleep(0.005)
            
            #print("Q: %s" % q)
            #print("W: %s" % w)
            
            return np.array([x, y, s*u, s*v])

    def setRollPitchFromAccels(self):
        # self.a is a np.array with three elements: gravitation accel in the body frame
        # for each axis. This points to gravity when Prudentia is not moving.
        # This function will only be called if we aren't moving, so don't worry
        # about checking that. 
        
        # Use self.a to return pitch and roll
        a_x = self.a[0]
        a_y = self.a[1]
        a_z = self.a[2]
        pitch = a_x/sqrt(a_y*a_y+a_z*a_z) 
        roll = a_y/sqrt(a_x*a_x+a_z*a_z)
        results = {}
        results['p'] = np.arctan2(-a_x,sqrt(a_y*a_y+a_z*a_z))
        results['r'] = np.arctan2(a_y,sqrt(a_x*a_x+a_z*a_z))
        return results

if __name__ == "__main__":

    np.set_printoptions(precision=4)

    Imu = ImuSingleton()
    Imu.a = np.array([0,0,-9.81])
    x = Imu.getRandomQuat  
    results = Imu.setRollPitchFromAccels()
    print("Roll: %s, Pitch: %s" % (results['r']*180/3.1415, results['p']*180/3.1415))


    #conn = Imu.openConnection('/dev/ttyUSB0', 115200) # Raspi USB
    #conn = Imu.openConnection('com13', 115200) # Windows USB
    #assert conn is not None

    #Imu.q = Imu.getRandomQuat()

    #Imu.w = np.array([0.0, 0.0, 0.1], dtype=float)


    #serialThread = Thread(target=Imu.emulateImu)
    #serialThread.start()
    #serialThread.join()
    #while True:
        #time.sleep(0.05)
        #print("Q:%s, W:%s" % (Imu.q, Imu.w))
        
