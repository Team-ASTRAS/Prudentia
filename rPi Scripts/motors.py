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
    
    accelLimit = 150 #rad/s^2. Accel of 246.53 rad/s^2 caused a back EMF overvoltage of 60V. 63V+ overvoltage may cause capacitor explosion.
    
    lastLoopTime = time.time()
    
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
        if len(deltaRpmArray) != 4:
            log("Error! setAllMotorRpm expected argument of length 4, got %s instead" % len(motorArray))
            return
        
        for i in range(len(deltaRpmArray)):
            deltaW = deltaRpmArray[i] * 2 * np.pi / 60
            
            if abs() > self.accelLimit:
                log("Requested RPM change in motor %s exceeded rpmRampLimit (%s/%s). Throttling input RPM." %
                    (i,deltaRpmArray[i], self.rpmRampLimit)) 
                deltaRpmArray[i] = self.rpmRampLimit
            
        deltaDutyArray = [0]*4
        
        for i in range(len(self.vescs)):
            
            newRpm = self.currentRpm[i] + deltaRpmArray[i]
            
            newDuty = self.getDutyFromRpm(newRpm)
            
            if newDuty > 0.8 or newDuty < -0.8:
                log("WARNING - Motor %s may be oversatured. New duty cycle request ignored: Over 80%% limit(%s%%)." % (i, round(newDuty*100, 2)))
            else:
                self.targetRpm[i] = newRpm
                self.duty[i] = newDuty
                
            self.vescs[i].set_duty_cycle(self.duty[i])
            deltaDutyArray[i] = self.duty[i]
        
        return deltaDutyArray

    #This function converts desired rpm change to duty cycle change
    def getDutyFromRpm(self, deltaRpm):
        return (deltaRpm*10 + 267.34) / 524.55 / 100.0

if __name__ == "__main__":
    Motors = MotorsSingleton()
    
    Motors.setAllMotorRpm([1000, 0, 0, 0])
    print(Motors.currentDC)
    time.sleep(2)
    
    Motors.setAllMotorRpm([1000, 0, 0, 0])
    print(Motors.currentDC)
    time.sleep(2)
    
    Motors.setAllMotorRpm([1000, 0, 0, 0])
    print(Motors.currentDC)
    time.sleep(2)
    
    Motors.setAllMotorRpm([1000, 0, 0, 0])
    print(Motors.currentDC)
    time.sleep(2)
    
    Motors.setAllMotorRpm([1000, 0, 0, 0])
    print(Motors.currentDC)
    time.sleep(2)
    