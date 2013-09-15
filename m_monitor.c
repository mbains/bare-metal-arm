
//
// Detects motion from PIR connected on GPIO Port D, pin0 aka PTD0, J2-6
//

#include <stdio.h>
#include <stdlib.h>
#include "freedom.h"
#include "common.h"
#include "gpio.h"
#include "spi.h"

#define COMMAND_SIZE (4)

// Main program
int main(void)
{
    char buf[COMMAND_SIZE+2]; //
    int idx;
    // Initialize all modules
    uart_init(9600);
   
    setvbuf(stdin, NULL, _IONBF, 0);        // No buffering

    // Run tests
    tests();
    delay(500);
    gpio_input_enable(FRDM_GPIO_PORT_D, 5, FRDM_IRQC_RISING_EDGE);
    PORTD_PCR0 = PORT_PCR_MUX(1);
    PORTD_PCR1 = PORT_PCR_MUX(1);
    PORTD_PCR2 = PORT_PCR_MUX(1);
    PORTD_PCR3 = PORT_PCR_MUX(1);

    GPIOD_PDDR |= 0xf;
    // Welcome banner
    iprintf("\r\rFRDM\r\n");
  
    PTD_BASE_PTR->PSOR = 0xf;
    for(;;) {
        fgets(buf, COMMAND_SIZE+1, stdin);//fgets leaves room for null term
        
        //iprintf("%d, %d, %d, %d\r\n", buf[0], buf[1], buf[2], buf[3]);
        for(idx = 0; idx < COMMAND_SIZE; idx++)
        {
            switch(buf[idx]){
                case '0':
                    PTD_BASE_PTR->PDOR &= ~(1 << idx);
                    break;
                case '1':
                    PTD_BASE_PTR->PDOR |= (1 << idx);
                    break;
            }
        }
        iprintf("qry:%d\r\n", (int) getPortD_IRQ_count());

    }
}
