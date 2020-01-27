#include "mbed.h"
#include "rtos.h"

#define SRSG_LED0 P0_16
#define SRSG_LED1 P0_15
#define SRSG_LED2 P0_17
#define SRSG_LED3 P0_18

DigitalOut l0(SRSG_LED0);
DigitalOut l1(SRSG_LED1);
DigitalOut l2(SRSG_LED2);
DigitalOut l3(SRSG_LED3);

#define SRSG_TX0 P0_2
#define SRSG_RX0 P0_3
Serial pc(SRSG_TX0,SRSG_RX0);
#define SRSG_TX1 P2_0
#define SRSG_RX1 P2_1
RawSerial s1(SRSG_TX1,SRSG_RX1);

void blink1(void) {
    int state=0;//0:serch 1st 90, 1:serch 2nd 90, 2:check size, 3:end
    unsigned char buf_data[128];
    int buf_size=0;
    while (1) {
        while(s1.readable()) {
            unsigned char rdata=s1.getc();
            if(state==0 || state==1){
                if(rdata==90)state++;
            }
            else if(state==2){
                //process
                state=0;
            }
        }
        Thread::wait(1);
    }
}
void blink2(void) {
    while (1) {
        l2=!l2;
        Thread::wait(1000);
    }
}
 
// main() runs in its own thread in the OS
// (note the calls to Thread::wait below for delays)
int main() {
    pc.baud(1000000);

    Thread thread1;
    thread1.start(blink1);
    Thread thread2;
    thread2.start(blink2);
    while (true) {
        l0 = !l0;
        Thread::wait(500);
    }
}

