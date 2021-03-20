import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

LED = 21
ledState = False
GPIO.setup(LED,GPIO.OUT)

ledState = not ledState
GPIO.output(LED, ledState)

while True:

    time.sleep(0.5)

    ledState = not ledState
    GPIO.output(LED, ledState)



