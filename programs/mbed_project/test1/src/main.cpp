#include "mbed.h"

#define SRSG_LED0 P0_16
#define SRSG_LED1 P0_15
#define SRSG_LED2 P0_17
#define SRSG_LED3 P0_18

DigitalOut l0(SRSG_LED0);
DigitalOut l1(SRSG_LED1);
DigitalOut l2(SRSG_LED2);
DigitalOut l3(SRSG_LED3);

int main() {
  while(1) {
/*
    l0=1;
    l1=0;
    l2=0;
    l3=0;
    wait(0.5);
    l0=0;
    l1=1;
    l2=0;
    l3=0;
    wait(0.5);
    l0=0;
    l1=0;
    l2=1;
    l3=0;
    wait(0.5);
    l0=0;
    l1=0;
    l2=0;
    l3=1;
    wait(0.5);  
*/
    l0=1;
    l1=1;
    l2=0;
    l3=0;
    wait(0.5);
    l0=0;
    l1=0;
    l2=1;
    l3=1;
    wait(0.5);

  }
}

