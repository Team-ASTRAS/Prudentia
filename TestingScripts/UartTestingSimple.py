import serial
import time
from threading import Thread
from pyvesc import VESC

serialPorts = ['/dev/ttyAMA1', #TX-IDSD  #RX-IDSC
               '/dev/ttyAMA2', #TX-IO4   #RX-IO5
               '/dev/ttyAMA3', #TX-CE0   #RX-MISO
               '/dev/ttyAMA4'] #TX-IO12  #RX-IO13


vesc0 = VESC(serialPorts[0], baudrate=115200)

for i in range(3):

    print('forward')
    vesc0.set_duty_cycle(0.25)
    time.sleep(2)

    print('backward')
    vesc0.set_duty_cycle(-0.25)
    time.sleep(2)
    
    i = i+1

vesc0.set_duty_cycle(0)

