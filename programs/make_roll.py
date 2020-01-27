# -*- coding: utf-8 -*-
import math

print("OK")

def roll(my_pos,my_dir,target):
	my_dir_x=my_dir[0]/math.sqrt(my_dir[0]**2+my_dir[1]**2)
	my_dir_y=my_dir[1]/math.sqrt(my_dir[0]**2+my_dir[1]**2)
	diff_x=target[0]-my_pos[0]
	diff_y=target[1]-my_pos[1]
	x=float(diff_x*my_dir_x+diff_y*my_dir_y)
	y=float(diff_y*my_dir_x-diff_x*my_dir_y)
	if x<=0:
		return False,0,0
	elif y==0:
		curve=0#realy infinity
		theta=0
		return True,0,x
	else:
		curve=(x**2+y**2)/(2*y)
		theta=math.atan2(x,math.fabs(curve-y))
		return True,curve,math.fabs(curve*theta)


print(roll([0,0],[1,0],[-1,0.00001]))
