#!/usr/bin/env python
import serial
import time

ser = serial.Serial()
#ser.port = '/dev/ydlidar'
ser.port = '/dev/ttyUSB0'
ser.baudrate = 9600
ser.timeout = 0.1
ser.open()

time.sleep(1.5)

print("DTR:1")
ser.setDTR(1)
time.sleep(5)

print("DTR:0")
ser.setDTR(0)
time.sleep(5)

print("DTR:1")
ser.setDTR(1)
time.sleep(5)

#ser.write(gen_fe(12,'W','S',5,value*100))
#strs = ser.read(100)

