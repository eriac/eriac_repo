#!/usr/bin/env python
import serial
import time

ser = serial.Serial()
ser.port = '/dev/ttyUSB0'
ser.baudrate = 9600
ser.timeout = 0.1
ser.open()
time.sleep(0.5)

output_data=[170,16,10]
ser.write(output_data)

time.sleep(0.5)

strs=""
frame=0
for step in range(100):
    strs += ser.read(1000)
    #for i in range(len(strs)):
        
    print(len(strs)/14)
    
