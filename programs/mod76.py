# -*- coding: utf-8 -*-
import math

print("x = x^2 (mod 100)")
for x in range(10,100):
	if((x * x)%100 == x):
		print(x)

print("%4 %25")
for x in range(10,100):
	if(x % 4 == 0 or x % 4 == 1):
		if(x % 25 == 0 or x % 25 == 1):
			print(x)

