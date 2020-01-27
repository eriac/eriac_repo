# -*- coding: utf-8 -*-
import math

def getN(use_list, num):
	count=-1
	for i in range(len(use_list)):
		if use_list[i]==False:
			count+=1
		if count==num:
			use_list[i]=True
			return i
	return -1

index=0
for i0 in range(0,7):
	for i1 in range(i0+1,8):
		for i2 in range(i1+1,9):
			for i3 in range(i2+1,10):
				print(index,":",i0, i1, i2, i3)
				#use_list=[False for i in range(10)]
				#print(getN(use_list, i0), end=",")
				#print(getN(use_list, i1), end=",")
				#print(getN(use_list, i2), end=",")
				#print(getN(use_list, i3))
				index+=1
