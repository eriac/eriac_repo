#!/usr/bin/env python
import serial
import time

ser = serial.Serial()
ser.port = '/dev/led'
ser.baudrate = 9600
ser.timeout = 0.1
ser.open()
time.sleep(1.5)


def gen_fe(station,command,group,funcid,data):
	output_data=[0 for _ in range(16)]
	output_data[ 0]=1
	output_data[ 1]=ord(str(station).zfill(2)[0])
	output_data[ 2]=ord(str(station).zfill(2)[1])
	output_data[ 3]=5
	output_data[ 4]=ord(command)
	output_data[ 5]=ord(group)
	output_data[ 6]=ord(str(funcid).zfill(2)[0])
	output_data[ 7]=ord(str(funcid).zfill(2)[1])
	output_data[ 8]=ord(" ")
	output_data[ 9]=ord(format(data, 'X').zfill(4)[0])
	output_data[10]=ord(format(data, 'X').zfill(4)[1])
	output_data[11]=ord(format(data, 'X').zfill(4)[2])
	output_data[12]=ord(format(data, 'X').zfill(4)[3])
	output_data[13]=3
	sum0=0
	for i in range(13):
		sum0+=output_data[i+1]
	output_data[14]=ord(format(sum0, 'x').zfill(3)[1])
	output_data[15]=ord(format(sum0, 'x').zfill(3)[2])

	return output_data
	
def set_hz(value):
	print(    gen_fe(12,'W','S',5,value*100))
	ser.write(gen_fe(12,'W','S',5,value*100))
	strs = ser.read(100)

	output=""
	for i in strs:
		if(ord(i)<0x20):
			output+="["+str(ord(i))+"]"
		else:
			output+=i
	print(output)

set_hz(20)

