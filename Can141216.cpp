#include "mbed.h"

//#define CANMBED
#ifdef CANMBED

Serial pc(USBTX, USBRX); 

Ticker ticker;
DigitalOut led1(LED1);
CAN can_port (p30, p29);
char counter = 0;

void send()
{
    if(can_port.write(CANMessage(1337, &counter, 1))) {
        counter++;
        //pc.printf("Message sent: %d\n", counter);
        led1 = !led1;
    }
}
int main()
{
    pc.printf("main()\n");
    ticker.attach(&send, 0.1);
    while(1) {
        printf("loop()\n");
    }
}
#endif //CANMBED