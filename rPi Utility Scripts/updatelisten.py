import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
import time
import subprocess

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering

pin = 10 #GPIO pin to detect button press
pressThreshold = 2.0 # Minimum seconds between presses (debouncer)

GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# Set pin to be an input pin and set initial value to be pulled low (off)

ready = True

while True: # Run forever
    if GPIO.input(pin) == GPIO.HIGH and ready:
        ready = False        
        readyTimer = time.time()
        print("Update pushed! Calling updaterepo.sh")
        p = subprocess.call(['x-terminal-emulator', '-e', 'bash updaterepo.sh'])

    if not ready and (time.time() - readyTimer > pressThreshold):
        ready = True
    
    time.sleep(0.01)
    



