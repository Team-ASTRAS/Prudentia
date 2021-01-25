import serial
import random

class ImuSingleton:

    #Serial connection attributes
    port = 'com4'
    baudrate = 921600
    conn = None

    #Imu data fields. These are updated by self.asyncRead() on another thread

    position = [0, 0, 0]
    velocity = [0, 0, 0]


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
        while True:
            serial.time.sleep(0.004)
            self.position[0] += random.randrange(-10,10, 1) / 10
            self.position[0] = max(min(self.position[0], 180), -180)
            self.position[1] += random.randrange(-10,10, 1) / 10
            self.position[1] = max(min(self.position[1], 180), -180)
            self.position[2] += random.randrange(-10,10, 1) / 10
            self.position[2] = max(min(self.position[2], 180), -180)

            #out = ''
            #while self.conn.inWaiting() > 0:
            #    out += self.conn.read(1)

            #if out != '':
            #    #Process data
            #    print(">>" + out)


