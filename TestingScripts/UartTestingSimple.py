import serial
import time
from threading import Thread
from pyvesc import VESC

serialPorts = ['/dev/ttyAMA1', #TX-IDSD  #RX-IDSC
               '/dev/ttyAMA2', #TX-IO4   #RX-IO5
               '/dev/ttyAMA3', #TX-CE0   #RX-MISO
               '/dev/ttyAMA4'] #TX-IO12  #RX-IO13



for i in range(4):
    
    vesc = VESC(serialPorts[i], baudrate=115200)
    print("------ Testing Motor %s ------" % i)

    print('forward')
    vesc.set_duty_cycle(0.10)
    time.sleep(2)
    
    print('stop')
    vesc.set_duty_cycle(0)
    time.sleep(2)

    print('backward')
    vesc.set_duty_cycle(-0.10)
    time.sleep(2)
    
    print('stop')
    vesc.set_duty_cycle(0)
    time.sleep(2)

