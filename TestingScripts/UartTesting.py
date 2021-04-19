import serial
import time
from threading import Thread
from pyvesc import VESC

#Define serial ports

serialPorts = ['/dev/ttyAMA1', #TX-IDSD  #RX-IDSC
               '/dev/ttyAMA2', #TX-IO4   #RX-IO5
               '/dev/ttyAMA3', #TX-CE0   #RX-MISO
               '/dev/ttyAMA4'] #TX-IO12  #RX-IO13

vescs = [None, None, None, None]

#Open serial ports 
for i in range(len(serialPorts)):
    vescs[i] = VESC(serialPorts[i], baudrate=115200)
    print("Is port %s open? : %s" % (serialPorts[i], vescs[i].serial_port.is_open))

increasing = True
dc = 0.02

startTime = time.time()


def getDutyAsync(vesc, rpmObj):
    while True:
        time.sleep(0.001)
        rpmObj.rpm = vesc.get_rpm() / 10.0
      
class rpmObj:
    rpm = 0
    
RPM = rpmObj()

getDutyThread = Thread(target=getDutyAsync, args=(vescs[3], RPM ))
getDutyThread.start()

timeNow = time.time()

c = 0
allowedTime = 1/20

dutyCyclePerStep = 0.001  #was 0.008


while timeNow - startTime < 300:
    c = c + 1
    
    if dc >= 0.80 and increasing:
        increasing = False
    if dc <= -0.80 and not increasing:
        increasing = True
        
    if increasing:
        dc = dc + dutyCyclePerStep
    else:
        dc = dc - dutyCyclePerStep
        
    dc = round(dc,3)
    
    timeNow = time.time() - startTime
    targetTime = c * allowedTime
    
    
    for i in range(4):
        if i == 3:
            print("Time:[%s], RPM:[%s], DC:[%s]" % (round(timeNow, 4), RPM.rpm, dc))
            f = open("testData.csv", "a")
            f.write("%s,%s,%s\n" % (round(timeNow, 4), RPM.rpm, dc))
            f.close()
        vescs[i].set_duty_cycle(dc)
        #print(dc)
    
    

    if timeNow > targetTime:
        log("Loop time exceeded allowance. Loop time: %s. Allowed time: %s" %
            (loopTime, allowedTime))
    else:

        sleepTime = targetTime - timeNow
        time.sleep(sleepTime) # Sleep for remaining loop time

print("END TEST")
for vesc in vescs:
    if vesc is not None:
        pass#vesc.set_duty_cycle(0)



 
 