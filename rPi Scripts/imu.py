import time
from time import sleep

import serial
from threading import Thread

class ImuSingleton:

    #Serial connection attributes
    port = ''
    baudrate = 0
    conn = None
        
    #Imu data fields. These are updated by self.asyncRead() on another thread
    

    def openConnection(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        try:
            self.conn = serial.Serial(self.port, self.baudrate)
            return self.conn
        except:
            print("Failed to open port %s " % str(self.port))

    def getAverageData(self):
        dataOut = [0] * 9
        dataOut[0] = self.avg(self.yaw)
        dataOut[1] = self.avg(self.pitch)
        dataOut[2] = self.avg(self.roll)
        dataOut[3] = self.avg(self.gyroX)
        dataOut[4] = self.avg(self.gyroY)
        dataOut[5] = self.avg(self.gyroZ)
        dataOut[6] = self.avg(self.accX)
        dataOut[7] = self.avg(self.accY)
        dataOut[8] = self.avg(self.accZ)
        return dataOut

    def avg(self, array):
        return sum(array) / len(array)

    def report(self):
        print("YPR - [%s,%s,%s], GYRO - [%s, %s, %s], ACCEL - [%s, %s. %s]" %
              (self.avg(self.yaw), self.avg(self.pitch), self.avg(self.roll),
               self.avg(self.gyroX), self.avg(self.gyroY), self.avg(self.gyroZ),
               self.avg(self.accX), self.avg(self.accY), self.avg(self.accZ)))

    def literalToFloat(self, literal):
        try:
            returnValue = float(literal)
            return returnValue

        except ValueError:
            try:
                array = literal.split('*')
                returnValue = float(array[0]) * int(array[1], 16)
                return returnValue
            except:
                return None

    #This will update the imu data fields.
    def asyncRead(self):
        conn.reset_input_buffer()
        output = " "
        i = 0
        while True:
            #print("---")
            t0 = time.time()
            while output != "":
                output = conn.readline().decode('ascii')
                
                splitOutput = output.strip().split(',')
                
                if len(splitOutput) > 12:
                    dataType = splitOutput[0]
                    if dataType == "$VNYMR":
                        self.yaw[i] = float(splitOutput[1])
                        self.pitch[i] = float(splitOutput[2])
                        self.roll[i] = float(splitOutput[3])

                        self.accX[i] = float(splitOutput[7])
                        self.accY[i] = float(splitOutput[8])
                        self.accZ[i] = float(splitOutput[9])

                        self.gyroX[i] = float(splitOutput[10])
                        self.gyroY[i] = float(splitOutput[11])
                        self.gyroZ[i] = self.literalToFloat(splitOutput[12])

                    else:
                        print("VN Code %s not supported!" % dataType)
                    
                    i += 1
                    if i >= (self.imuSpeed / 20):
                        i = 0
                        
                else:
                    print("ERROR: splitOutput length is not greater than zero!")

            print(time.time() - t0)
            #Reset output
            output = " "
                

if __name__ == "__main__":
    Imu = ImuSingleton()
    #conn = Imu.openConnection('/dev/ttyUSB0', 460800) # Raspi USB
    conn = Imu.openConnection('com13', 115200) # Windows USB
    assert conn is not None
    serialThread = Thread(target=Imu.asyncRead)
    serialThread.start()

    i = 0
    t0 = time.time()
    while i < 1000000:
        i += 1
        arr = Imu.getAverageData()
        Imu.report()
    t = time.time() - t0
    print(t)