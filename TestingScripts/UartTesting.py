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


def getDutyAsync(vesc):
    while True:
        time.sleep(0.001)
        print(vesc.get_rpm())
        
getDutyThread = Thread(target=getDutyAsync, args=(vescs[0], ))
getDutyThread.start()



while time.time() - startTime < 30:
    if dc >= 0.80 and increasing:
        increasing = False
    if dc <= -0.80 and not increasing:
        increasing = True
        
    if increasing:
        dc = dc + 0.001
    else:
        dc = dc - 0.001
        
    dc = round(dc,3)
    
    for i in range(4):
        vescs[i].set_duty_cycle(dc)
    
    
    currentTime = time.time() - startTime
    print("[%s] - OUT: %s" % (currentTime, dc) )
    
    time.sleep(0.003)
    

print("END TEST")
for vesc in vescs:
    if vesc is not None:
        pass#vesc.set_duty_cycle(0)



 
 