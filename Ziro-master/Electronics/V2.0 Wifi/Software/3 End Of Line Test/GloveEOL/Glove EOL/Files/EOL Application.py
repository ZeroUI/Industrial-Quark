import os
import serial
import glob


for port in range(256):
	try:
		s=serial.Serial(str(port))
		s.close()
		result.append(port)
	except(OSError,serial.SerialException):
		 pass

for p in result:
	print p