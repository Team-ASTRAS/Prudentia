import serial

port = serial.Serial('/dev/ttyS0', baudrate=9600)
port.write("YOO")

 
 
