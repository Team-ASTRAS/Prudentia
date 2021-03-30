import serial
import time
from threading import Thread
from pyvesc import VESC

serialPorts = ['/dev/ttyAMA1', #TX-IDSD  #RX-IDSC
               '/dev/ttyAMA2', #TX-IO4   #RX-IO5
               '/dev/ttyAMA3', #TX-CE0   #RX-MISO
               '/dev/ttyAMA4'] #TX-IO12  #RX-IO13


vesc0 = VESC(serialPorts[0], baudrate=115200)
vesc1 = VESC(serialPorts[1], baudrate=115200)
vesc2 = VESC(serialPorts[2], baudrate=115200)
vesc3 = VESC(serialPorts[3], baudrate=115200)
def setDutyThread(vesc0):
    while True:
        time.sleep(0.05)
        t0 = time.time()
        print(vesc0.get_rpm())
        t1 = time.time()
        print("Time taken: %s" % (t1-t0))
        
getDutyThread = Thread(target=setDutyThread, args=(vesc0, ))
getDutyThread.start()


for i in range(3 ):

    print('forward')
    vesc0.set_rpm(500)
    vesc1.set_duty_cycle(0.10)
    vesc2.set_duty_cycle(-0.10)
    vesc3.set_duty_cycle(0.10)
    time.sleep(2)

    print('backward')
    vesc0.set_rpm(500)
    vesc1.set_duty_cycle(-0.10)
    vesc2.set_duty_cycle(0.10)
    vesc3.set_duty_cycle(-0.10)

    time.sleep(2)
    
    i = i+1

print('end')
vesc0.set_duty_cycle(0)
vesc1.set_duty_cycle(0)
vesc2.set_duty_cycle(0)
vesc3.set_duty_cycle(0)

