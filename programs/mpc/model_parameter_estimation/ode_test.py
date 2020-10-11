# coding: utf-8

import numpy as np #numpy
from scipy.integrate import odeint #odeint
import matplotlib.pyplot as plt #to draw graphs

def func(s, t, r, m):
    y, v = s #sは変数yとvの組
    g=9.80665 #m/s^2
    dsdt = [v, -r*v/m - g]
    # dsdt = [v, (-r*v-m*g)/m]
    return dsdt

r=100
m=100
y0 = [0,10]#位置0から初速10で投げ上げ
t = np.linspace(0, 7, 201)#時刻0から7まで、201step刻みで計算する

sol = odeint(func, y0, t, args=(r,m))

plt.plot(t, sol[:, 0], 'b', label='y')#yについてplot
plt.plot(t, sol[:, 1], 'g', label='v')#vについてplot
plt.legend(loc='best')#レジェンドを付ける
plt.xlabel('t')
plt.grid()#格子を付ける
plt.show()