import serial
from threading import Thread
from queue import Queue

class ImuSingleton:

    #Serial connection attributes
    port = ''
    baudrate = 0
    conn = None

    #Imu data fields. These are updated by self.asyncRead() on another thread

    yaw = [0] * 10
    pitch = [0] * 10
    roll = [0] * 10
    
    gyroX = [0] * 10
    gyroY = [0] * 10
    gyroZ = [0] * 10
    
    accX = [0] * 10
    accY = [0] * 10
    accZ = [0] * 10
    


    def openConnection(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        try:
            self.conn = serial.Serial(self.port, self.baudrate)
            return self.conn
        except:
            print("Failed to open port %s " % str(self.port))

    #This will update the imu data fields.
    def asyncRead(self):
        conn.reset_input_buffer()
        output = " "
        i = 0
        while True:
            print("---")
            while output != "":
                output = conn.readline().decode('ascii')
                
                splitOutput = output.split(',')
                
                if len(splitOutput) > 12:
                    dataType = splitOutput[0]
                    if dataType == "$VNYMR":
                        self.yaw[i] = splitOutput[1]
                        self.pitch[i] = splitOutput[2]
                        self.roll[i] = splitOutput[3]
                        
                        self.gyroX[i] = splitOutput[7]
                        self.gyroY[i] = splitOutput[8]
                        self.gyroZ[i] = splitOutput[9]
                        
                        self.accX[i] = splitOutput[10]
                        self.accY[i] = splitOutput[11]
                        self.accZ[i] = splitOutput[12]
                        
                        print("YPR - [%s,%s,%s], GYRO - [%s, %s, %s], ACCEL - [%s, %s. %s]" %
                              (self.yaw[i], self.pitch[i], self.roll[i],
                               self.gyroX[i], self.gyroY[i], self.gyroZ[i],
                               self.accX[i], self.accY[i], self.accZ[i]) )
                    else:
                        print("VN Code %s not supported!" % dataType)
                    
                    i += 1
                    if i >= 10:
                        i = 0
                        
                else:
                    print("ERROR: splitOutput length is not greater than zero!")
                
            #Reset output
            output = " "
                


Imu = ImuSingleton()
conn = Imu.openConnection('/dev/ttyUSB0', 460800)
assert conn is not None
serialThread = Thread(target=Imu.asyncRead)
serialThread.start()

