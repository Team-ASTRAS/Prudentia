import serial
import time
from pyvesc import VESC

serialPorts = ['/dev/ttyAMA1', #TX-IDSD  #RX-IDSC
               '/dev/ttyAMA2', #TX-IO4   #RX-IO5
               '/dev/ttyAMA3', #TX-CE0   #RX-MISO
               '/dev/ttyAMA4'] #TX-IO12  #RX-IO13


vesc = VESC(serialPorts[0], baudrate=9600)

for i in range(1000):

    print('forward')
    vesc.set_duty_cycle(0.05)

    time.sleep(2)

    print('backward')
    vesc.set_duty_cycle(-0.05)

    time.sleep(2)

print('end')
vesc.set_duty_cycle(0)
