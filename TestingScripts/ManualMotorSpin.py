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



currentRpm = [0, 0, 0, 0]
targets = [-0.9, 0.0, 0.9, 0.0]
#[0, +, 0, -] => -z
#[0, +, 0, -] => +z
setRpms = [0, 0, 0, 0]
step = 0.0015
startTime = time.time()


while True:
    time.sleep(0.005)
    for i in range(4):
        if targets[i] > currentRpm[i]:
            if abs(targets[i] - currentRpm[i]) < step:
                setRpm = targets[i]
            else:
                setRpm = currentRpm[i] + step
            setRpms[i] = setRpm
        elif targets[i] < currentRpm[i]:
            if abs(targets[i] - currentRpm[i]) < step:
                setRpm = targets[i]
            else:
                setRpm = currentRpm[i] - step
            setRpms[i] = setRpm
    currentRpm = setRpms
    print(currentRpm)
    for i in range(4):
        vescs[i].set_duty_cycle(currentRpm[i])



 
 