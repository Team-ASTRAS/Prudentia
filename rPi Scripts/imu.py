import time
import struct
import numpy as np
from math import sqrt, cos, sin
from time import sleep

import serial
from controlLaw import eulerAxis2quat, quatMultiply, ypr2str, quat2ypr, w2str, normalizeQuat
from threading import Thread
from random import uniform

#$VNWRG,75,1,16,04,000C*XX
#^Binary Output Group Command^

class ImuSingleton:

    #Serial connection attributes
    port = ''
    baudrate = 0
    conn = None
    packetLength = 30    
    updateFrequency = 50 #Hz
    
    gyroOffsets = np.array([0.000745, 0.000729, 0.000469], dtype=float)
        


    #Imu data fields. These are updated by self.asyncRead() on another thread
    q = np.array([0, 0, 0, 1], dtype=float)
    w = np.array([0, 0, 0], dtype=float)
    a = np.array([0, 0, 0], dtype=float)
    
    #Packets start with identifier "FA 04 0C 00"
    startBytes = bytearray([int('0xfa', 16), int('0x04', 16), int('0x0C', 16), int('0x00', 16)])

    def openConnection(self, port, baudrate):
        self.port = port
        self.q = normalizeQuat(self.q)
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
                        packet = output[0:self.packetLength]
                        #print("Packet data (%s): %s" % (len(packet), packet))
                        #print("All data (%s): %s" % (len(output), output))
                        self.processPacket(packet)
                        del output[0:self.packetLength]
                    
    def processPacket(self, packet):
        i = 4
        self.a[0] = struct.unpack('f', packet[i:i+4])[0]
        i+=4
        self.a[1] = struct.unpack('f', packet[i:i+4])[0]
        i+=4
        self.a[2] = struct.unpack('f', packet[i:i+4])[0]

        i+=4
        self.w[0] = struct.unpack('f', packet[i:i+4])[0] 
        i+=4
        self.w[1] = struct.unpack('f', packet[i:i+4])[0]
        i+=4
        self.w[2] = struct.unpack('f', packet[i:i+4])[0]
        
        self.w[0] = self.w[0] + 0.21
        self.w[1] = self.w[1] + 0.001
        self.w[2] = self.w[2] - 0.008


        #print("W: %s" % self.w)

        #self.w[0] = self.w[0] + self.gyroOffsets[0]

        self.propogateQuaternion()

    def propogateQuaternion(self):
        w = np.deg2rad(self.w) / self.updateFrequency
        wMag = np.linalg.norm(w)
        wAxis = w / wMag

        if(wMag < 0.01):
            pass #TODO Recalibrate P, R

        deltaQ = eulerAxis2quat(wAxis, wMag * 180 / np.pi)
        self.q = quatMultiply(self.q, deltaQ)
        self.q = normalizeQuat(self.q)
    
    def setCalibration(self):
        print("Calibrating! Accels: %s" % self.a)

    def emulateImu(self):
        while True:
            sleep(0.02)

            self.w = 100 * np.array([uniform(-0.1,0.1), uniform(-0.1,0.1), uniform(-0.1,0.1)])
                    
            self.propogateQuaternion()
            #print("Propogation Data: %s      w: %s" % (ypr2str(quat2ypr(Imu.q)), w2str(self.w)))
                   
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

    def getRollPitch(self):
        # self.a is a np.array with three elements: gravitation accel in the body frame
        # for each axis. This points to gravity when Prudentia is not moving.
        # This function will only be called if we aren't moving, so don't worry
        # about checking that. 
        
        # Use self.a to return pitch and roll
        a_x = self.a[0]
        a_y = self.a[1]
        a_z = self.a[2]
        pitch = a_x/sqrt(a_y*a_y+a_z*a_z) 
        roll = -a_y/sqrt(a_x*a_x+a_z*a_z)
        results = {}
        results['p'] = np.arctan2(a_x,sqrt(a_y*a_y+a_z*a_z)) #only 0 to 90 rn, needs to be updated
        results['r'] = np.arctan2(-a_y,-a_z)
        return results

if __name__ == "__main__":

    np.set_printoptions(precision=4, suppress=True)

    Imu = ImuSingleton()
    #conn = Imu.openConnection('/dev/ttyUSB0', 115200) # Raspi USB
    conn = Imu.openConnection('com15', 115200) # Windows USB
    assert conn is not None

    serialThread = Thread(target=Imu.asyncRead)
    serialThread.start()
    while True:
        time.sleep(0.05)
        #results = Imu.getRollPitch()
        #print("Roll: %s, Pitch: %s" % (results['r']*180/3.1415, results['p']*180/3.1415))
        ypr = quat2ypr(Imu.q)
        print("Quaternion: %s" % Imu.q)
        print("IMU Euler: [%s, %s, %s]" % (ypr.y, ypr.p, ypr.r))

    serialThread.join()
        
