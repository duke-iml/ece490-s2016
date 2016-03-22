# Test of the Python <-> Arduino serial interface
# If this works correctly, you should see the orange light on the
# Arduino flash on and off with a period of 2 seconds
import serial
import time
ser = serial.Serial()
ser.port = "/dev/ttyACM0"
ser.baudrate = 9600
ser.open()
if ser.isOpen():
	ser.write("hello")
	response = ser.read(ser.inWaiting())
counter = 1
on = True
while True:
	counter = counter + 1
	if on:
		ser.write('L')
		on = False
	else:
		ser.write('H')
		on = True
	time.sleep(2)

	if counter>15:
		break


