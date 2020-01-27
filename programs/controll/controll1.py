#!/usr/bin/env python
from control.matlab import *
from matplotlib import pyplot as plt
from scipy import arange 

#sudo apt-get install python-pip
#sudo apt-get install gfortran
#sudo apt-get install libblas-dev libatlas-dev liblapack-dev
#pip install control
#pip install slycot

def main():
    CONTROLLER_G=1.0 # controller gain
    ACTUATOR_D =0.2 #actuator dead time
    ACTUATOR_P =0.2 #actuator primaly delay
    SENSOR_D=0.5 #sensor dead time 

    num = [CONTROLLER_G] 
    den = [1]
    ctrl = tf(num, den) 
    
    num = [0, 1] 
    den = [ACTUATOR_P, 1]
    p1 = tf(num, den)
    num, den = pade(ACTUATOR_D, 5)
    p2 = tf(num, den) 
    plant= p1*p2

    num = [0, 1] 
    den = [1, 0]
    s_itg = tf(num, den)

    num, den = pade(SENSOR_D, 5)
    sensor = tf(num, den) 
    
    sys = feedback(ctrl*plant*s_itg, sensor)
    
    print sys
    (y1a, T1a) = step(sys, T = arange(0, 10, 0.01))
    plt.axhline(1, color="b", linestyle="--")
    plt.plot(T1a, y1a)
    plt.show()

if __name__ == "__main__":
  main()
