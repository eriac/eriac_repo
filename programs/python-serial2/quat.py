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


for step in range(100):
    strs=""
    for i in range(1):
        strs += ser.read(100)
    for i in range(len(strs)):
        if ord(strs[i])==90 and ord(strs[i+1])==90:
            print("")
            print("")
            print("90:"+str(ord(strs[i+0])))
            print("90:"+str(ord(strs[i+1])))
            print("A :"+str(ord(strs[i+2])))
            print("B :"+str(ord(strs[i+3])))
            c0=(ord(strs[i+4])*256+ord(strs[i+5]))
            if c0>=32768:
                c0-=65536
            c1=(ord(strs[i+6])*256+ord(strs[i+7]))
            if c1>=32768:
                c1-=65536
            c2=(ord(strs[i+8])*256+ord(strs[i+9]))
            if c2>=32768:
                c2-=65536
            c3=(ord(strs[i+10])*256+ord(strs[i+11]))
            if c3>=32768:
                c3-=65536
            
            c0/=10000.0
            c1/=10000.0
            c2/=10000.0
            c3/=10000.0

            print("size"+str(len(strs)-i))
            print("w:"+str(c0))
            print("x:"+str(c1))
            print("y:"+str(c2))
            print("z:"+str(c3))
            print("sum^2:"+str(c0**2+c1**2+c2**2+c3**2))
            d0=(ord(strs[i+12])>>6)&0x03
            d1=(ord(strs[i+12])>>4)&0x03
            d2=(ord(strs[i+12])>>2)&0x03
            d3=(ord(strs[i+12])>>0)&0x03
            print("D0:"+str(d0)+","+str(d1)+","+str(d2)+","+str(d3)) 
            print("D1:"+str(ord(strs[i+13])))
            break
    time.sleep(0.1)


'''
for i in range(100):
    strs = ser.read(100)
    data= [0 for j in range(256)]
    for rd in strs:
        data[ord(rd)]+=1
    print(data[0x90])
'''
''' 
for i in range(10):
    strs = ser.read(1000)
    output=""
    for j in strs:
        output+=str(ord(j))+","
    print(output)
'''
