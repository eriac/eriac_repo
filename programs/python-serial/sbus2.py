#!/usr/bin/env python
import serial
import time

def low_value(strs, index, bits):
	return int(ord(strs[index]))>>bits

def hegh_value(strs, index, bits):
	return int(ord(strs[index])) % (2**bits)

if __name__ == '__main__':
	ser = serial.Serial()
	ser.port = '/dev/ttyUSB0'
	ser.baudrate = 100000
	# ser.parity = serial.PARITY_ODD
	ser.parity = serial.PARITY_EVEN
	ser.stopbits = serial.STOPBITS_TWO
	ser.timeout = 0.1


	ser.open()
	time.sleep(1)

	counter =0
	while(True):

		while(True):
			strs = ser.read(1)
			if(ord(strs[0]) == 0x0f):
				break
			# elif(ord(strs[0]) == 0):
			# 	print("reject", hex(ord(strs[0])))
			# else:
			# 	print(hex(ord(strs[0])))
			
		strs = ser.read(24)
		output=""
		for i in strs:
			output+= str(ord(i))+","
		counter += 1
		if counter%10 == 0:
			input = [0 for i in range(4)]
			input[0] = hegh_value(strs,1,3) * 2**8 + int(ord(strs[0])) # 3 8
			input[1] = (int(ord(strs[2])) % 64) * 32 + (int(ord(strs[1]))>>3) # 6 5 
			input[2] = (int(ord(strs[4])) % 2) * 1024 + (int(ord(strs[3])) % 256) * 4 + (int(ord(strs[2]))>>6) # 1 8 2
			input[3] = (int(ord(strs[5])) % 16) * 128 + (int(ord(strs[4]))>>1) # 4 7
			print(input) 
			# print(output)
