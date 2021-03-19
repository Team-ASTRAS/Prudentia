from utilities import log
from pyvesc import VESC


class MotorsSingleton:
    
    #Define serial ports
    serialPorts = ['/dev/ttyAMA1', '/dev/ttyAMA2', '/dev/ttyAMA3', '/dev/ttyAMA4']
    vescs = [None, None, None, None]
    dutys = [0, 0, 0, 0]
    
    def __init__(self):
        for i in range(len(self.serialPorts)):
            self.vescs[i] = VESC(self.serialPorts[i])
            log("Is VESC port %s open? : %s" % (self.serialPorts[i], self.vescs[i].serial_port.is_open))

    # This function expects an array of motor delta RPMs ex: [1000.2, -1820, 3000, -3000]
    def setAllMotorRpm(self, deltaRpmArray):
        if len(deltaRpmArray) != 4:
            log("Error! setAllMotorRP expected argument of length 4, got %s instead" % len(motorArray))
            return
        
        deltaDutyArray = [0]*4
        
        for i in range(len(self.vescs)):
            
            deltaDutyArray[i] = self.getDutyFromRpm(deltaRpmArray[i])
        
            newDuty = self.dutys[i] + deltaDutyArray[i]
            if newDuty > 0.8:
                pass
                #log("WARNING - Motor %s may be oversatured. New duty cycle request ignored: Over 80%% limit(%s%%)." % (i, round(newDuty*100, 2)))
            elif newDuty < 0.1:
                #log("CAUTION - Duty cycle low(%s)" % newDuty)
                self.dutys[i] = newDuty
            else:
                self.dutys[i] = newDuty
                
            self.vescs[i].set_duty_cycle(self.dutys[i])
            deltaDutyArray[i] = self.dutys[i]
        
        return deltaDutyArray

    #This function converts desired rpm change to duty cycle change
    def getDutyFromRpm(self, deltaRpm):
        return (deltaRpm + 267.34) / 524.55 / 100.0
