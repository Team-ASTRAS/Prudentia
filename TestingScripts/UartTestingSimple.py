import serial
import time
from threading import Thread
from pyvesc import VESC

serialPorts = ['/dev/ttyAMA1', #TX-IDSD  #RX-IDSC
               '/dev/ttyAMA2', #TX-IO4   #RX-IO5
               '/dev/ttyAMA3', #TX-CE0   #RX-MISO
               '/dev/ttyAMA4'] #TX-IO12  #RX-IO13


vesc = VESC(serialPorts[0], baudrate=115200)

def setDutyThread(vesc):
    while True:
        time.sleep(0.05)
        t0 = time.time()
        print(vesc.get_rpm())
        t1 = time.time()
        print("Time taken: %s" % (t1-t0))
        
getDutyThread = Thread(target=setDutyThread, args=(vesc, ))
getDutyThread.start()


for i in range(5):

    print('forward')
    vesc.set_duty_cycle(0.05)

    time.sleep(2)

    print('backward')
    vesc.set_duty_cycle(-0.05)

    time.sleep(2)

print('end')
vesc.set_duty_cycle(0)
