from utilities import log

class MotorsSingleton:

    pinMotor1 = 0
    pinMotor2 = 0
    pinMotor3 = 0
    pinMotor4 = 0

    # This function expects an array of motor RPMs ex: [1000.2, -1820, 3000, -3000]
    def setAllMotorRpm(self, motorArray):
        if len(motorArray) != 4:
            log("Error! setAllMotorRP expected argument of length 4, got %s instead" % len(motorArray))
            return


    #This function expects an integer motor (0, 1, 2, 3)
    def setMotorRpm(self, motor, rpm):
        pinMotor = 0
        if motor == 1:
            pinMotor = self.pinMotor1
        elif motor == 2:
            pinMotor = self.pinMotor2
        elif motor == 3:
            pinMotor = self.pinMotor3
        elif motor == 4:
            pinMotor = self.pinMotor4
        else:
            log("Error! Motor selection was outside range of 0 to 3")
            return

        duty = self.getDutyFromRPM(rpm)

        self.setPWM(pinMotor, duty)

    #This function converts desired rpm to duty cycle
    def getDutyFromRPM(self, rpm):
        # Return duty cycle needed to meet input rpm
        pass

    #This function sets a PWM pin to a specific duty cycle
    def setPWM(self, pin, duty):
        # Set pinout to designated duty cycle
        pass
