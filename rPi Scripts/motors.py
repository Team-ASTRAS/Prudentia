from utilities import log
from pyvesc import VESC
import numpy as np
from threading import Thread
import time

class MotorsSingleton:
    
    #Define serial ports
    serialPorts = ['/dev/ttyAMA1', '/dev/ttyAMA2', '/dev/ttyAMA3', '/dev/ttyAMA4']
    vescs = [None, None, None, None]
    duty = np.array([0, 0, 0, 0], dtype=float)
    currentRpm = np.array([0, 0, 0, 0], dtype=float)
    targetRpm = np.array([0, 0, 0, 0], dtype=float)
    
    #accelLimit = 150 #rad/s^2. Accel of 246.53 rad/s^2 caused a back EMF overvoltage of 60V. 63V+ overvoltage may cause capacitor explosion.
    dutyLimit = 0.165 #Max duty change per second
    loopSpeed = 20
    
    lastLoopTime = time.time()
    
    def resetMotors(self):
        for i in range(len(self.serialPorts)):
            ~self.vescs[i]
        for i in range(len(self.serialPorts)):
            self.vescs[i] = VESC(self.serialPorts[i])
            log("Is VESC port %s open? : %s" % (self.serialPorts[i], self.vescs[i].serial_port.is_open))
    
    def __init__(self):
        for i in range(len(self.serialPorts)):
            self.vescs[i] = VESC(self.serialPorts[i])
            log("Is VESC port %s open? : %s" % (self.serialPorts[i], self.vescs[i].serial_port.is_open))
        
        for i in range(len(self.serialPorts)):
            getDutyThread = Thread(target=self.setDutyThread, args=(self.vescs, i))
            getDutyThread.start()
    
    def setDutyThread(self, vescs, i):
        while True:
            time.sleep(0.02)
            self.currentRpm[i] = self.vescs[i].get_rpm() / 10.0
            #print("%s: %s" % (i, self.currentDC[i] ))

    # This function expects an array of motor delta RPMs ex: [1000.2, -1820, 3000, -3000]
    def setAllMotorRpm(self, deltaRpmArray):
        
        self.targetRpm = self.currentRpm + deltaRpmArray
        
        for i in range(len(self.vescs)):
            
            newDuty = self.getDutyFromRpm(self.targetRpm[i])
            
#             if abs(newDuty - self.duty[i]) > self.dutyLimit / self.loopSpeed:
#                 log("WARNING - Duty cycle limit of %s per loop reached. Capping duty cycle." % self.dutyLimit)
#                 newDuty = self.dutyLimit / self.loopSpeed
#             else:
                    
            if newDuty > 0.85 or newDuty < -0.85:
                log("WARNING - Motor %s is oversatured. New duty cycle request ignored: Over 85%% limit(%s%%)." % (i, round(newDuty*100, 2)))
            else:
                self.duty[i] = newDuty
                
            self.vescs[i].set_duty_cycle(self.duty[i])

    #This function converts desired rpm change to duty cycle change
    def getDutyFromRpm(self, rpm):
        #return (rpm*10 + 267.34) / 524.55 / 100.0 #OLD
        return 0.00019087 * rpm# + 0.026 #Setting y intercept to zero.

if __name__ == "__main__":
    Motors = MotorsSingleton()
    
    Motors.setAllMotorRpm(np.array([500, 500, 500, 500]))
    time.sleep(2)