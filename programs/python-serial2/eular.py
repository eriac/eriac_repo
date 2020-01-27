#!/usr/bin/env python
import serial
import time

ser = serial.Serial()
ser.port = '/dev/ttyUSB0'
ser.baudrate = 9600
ser.timeout = 0.1
ser.open()
time.sleep(0.5)

output_data=[170,8,10]
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
            #if c0>=32768:
            #    c0-=65536
            c1=(ord(strs[i+6])*256+ord(strs[i+7]))
            if c1>=32768:
                c1-=65536
            c2=(ord(strs[i+8])*256+ord(strs[i+9]))
            if c2>=32768:
                c2-=65536
            
            #print("C0:"+str((ord(strs[i+4])*256+ord(strs[i+5])^ 32768)-32768))
            #print("C1:"+str((ord(strs[i+6])*256+ord(strs[i+7])^ 32768)-32768))
            #print("C2:"+str((ord(strs[i+8])*256+ord(strs[i+9])^ 32768)-32768))
            print("yaw  :"+str(c0/100.0)+" deg")
            print("pitch:"+str(c1/100.0)+" deg")
            print("roll :"+str(c2/100.0)+" deg")
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
