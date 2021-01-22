import serial

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
        serial.time.sleep(1)
        out = ''
        while True:
            while self.conn.inWaiting() > 0:
                out += self.conn.read(1)

            if out != '':
                #Process data
                print(">>" + out)


