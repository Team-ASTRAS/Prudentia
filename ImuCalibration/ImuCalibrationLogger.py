import time
import serial
import os
from threading import Thread

#Supports up to 4 USB connections exposed by linux when connected
portNames = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3']
filePrefix = 'imu'
baudrate = 115200 
#50Hz

def readImuAsync(conn, i):
    while True:
        time.sleep(1)
        if conn is None:
            print("Connection is none! Breaking loop.")
            break
        
        try:
            output = conn.readline().decode('ascii')
            while output != "":
                
                time.sleep(0.001)
                #Print to console
                print("Reading IMU on port [%s]. Incoming Data: %s" % ("ttyUSB" + str(i), output))
                
                #Save line in CSV file.
                fileName = "IMU" + str(i) + '.csv'
                with open(fileName, 'a') as file:
                    file.write(output)
            
                #Check for multiple lines in serial buffer
                output = conn.readline().decode('ascii')
        except:
            pass


connections = [None, None, None, None]

#Open connections at start of program
for i in range(len(connections)):
    try:
        connections[i] = serial.Serial(portNames[i], baudrate)
        print("Scanning port \"%s\" at baudrate %s. Is open: %s" %
              (portNames[i], baudrate, connections[i] is not None))
    except:
        print("Port \"%s\" is not available." % (portNames[i]))

print("Starting data collection in 3 seconds...")
time.sleep(3)

start = time.time()

t = [None, None, None, None]

#readImuAsync(connections[0],0)

for i in range(len(connections)):
    if connections[i] is not None:
        t[i] = Thread(target=readImuAsync, args=(connections[i], i))
        t[i].start()
        print("Thread started for IMU %s." % i)

for i in range(len(t)):
    if t[i] is not None:
        t[i].join()
        