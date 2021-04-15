import time
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

switchPin = 27
GPIO.setup(switchPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
while True:
    isAlive = GPIO.input(switchPin)
    if isAlive:
       print("ALIVE")
    else:
        print("DEAD")
    time.sleep(0.2)
