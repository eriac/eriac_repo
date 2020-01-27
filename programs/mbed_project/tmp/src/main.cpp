#include "mbed.h"
#define SRSG_LED0 P0_16
#define SRSG_TX0 P0_2
#define SRSG_RX0 P0_3

DigitalOut l0(SRSG_LED0);
Serial pc(SRSG_TX0,SRSG_RX0);

int main() {
  pc.baud(1000000);
  while (true) {
    l0 = !l0;
    wait(500);
    pc.printf("hellow mbed!\n");
  }
}


