#!/usr/bin/env python
import serial
import time

def decode_bytes(strs):
	input = [0 for i in range(16)]
	for i in range(16):
		offset = i*11
		offset_byte = offset / 8
		offset_bit = offset %8
		if offset_bit < 5:
			input[i] = (int(ord(strs[offset_byte + 1])) % 2**(offset_bit+3)) * 2**(8-offset_bit) + int(ord(strs[offset_byte])>>offset_bit) # 3 8
		else:
			input[i] = (int(ord(strs[offset_byte+2])) % 2**(offset_bit-5)) * 2**(16-offset_bit) + int(ord(strs[offset_byte+1])) * 2**(8-offset_bit) + int(ord(strs[offset_byte])>>offset_bit) # 1 8 2
	return input

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

	names = ["" for i in range(16)]
	names[0] = "R-R"	
	names[1] = "L-D"	
	names[2] = "R-D"	
	names[3] = "L-R"	
	names[4] = "SwA"	
	names[5] = "SwB"	
	names[6] = "SwC"	
	names[7] = "SwD"	

	counter =0
	while(True):
		while(True):
			strs = ser.read(1)
			if len(strs) != 1:
				continue
			if(ord(strs[0]) == 0x0f):
				break
			
		strs = ser.read(24)
		if counter%10 == 0:
			ch_list = decode_bytes(strs)
			print("#######")
			# for i in range(16):
			# 	print(i,ch_list[i],names[i]) 
			print("x", -(ch_list[1]-1024)/660.0)
			print("y", -(ch_list[3]-1024)/660.0)