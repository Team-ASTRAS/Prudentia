import serial


serialPorts = ['/dev/ttyAMA1', #TX-IDSD  #RX-IDSC
               '/dev/ttyAMA2', #TX-IO4   #RX-IO5
               '/dev/ttyAMA3', #TX-CE0   #RX-MISO
               '/dev/ttyAMA4'] #TX-IO12  #RX-IO13

conn = serial.Serial(serialPorts[3])

testStr = "TEST\n"

conn.write(testStr.encode('ascii'))
b = conn.readline()
print(b)