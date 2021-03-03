import time
import serial
import os

#Supports up to 4 USB connections exposed by linux when connected
portNames = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3']
filePrefix = 'imu'
baudrate = 115200

connections = [None, None, None, None]

#Open connections at start of program
for i in range(len(connections)):
    if os.path.isfile(portNames[i]):
        connections[i] = serial.Serial(portNames[i], baudrate)
        print("Scanning port \"%s\" at baudrate %s. Is open: %s" %
              (portNames[i], baudrate, connections[i] is not None))
    else:
        print("Port \"%s\" is not available." % portNames[i])

print("Starting data collection in 3 seconds...")
time.sleep(3)

start = time.time()
while True:
    print('-' * 40)
    time.sleep(0.01)
    for conn in connections:
        if conn is not None:
            
            output = conn.readline().decode('ascii')
            while output != "":
                #Print to console
                print("Reading IMU on port [%s]. Incoming Data: %s" % (conn, output))
                
                #Save line in CSV file.
                fileName = conn + '.csv'
                with open(fileName, 'a') as file:
                    file.write(output + '\n')
                    
                #Check for multiple lines in serial buffer
                output = conn.readline().decode('ascii')