//
// demo.c -- Simple demonstration program
//
//  Copyright (c) 2012-2013 Andrew Payne <andy@payne.org>
//

#include <stdio.h>
#include <stdlib.h>
#include "freedom.h"
#include "common.h"
#include "gpio.h"
#include "spi.h"
#include "mrf24j40_frdm.h"

extern char *_sbrk(int len);

// Main program
int main(void)
{
    char i;
    char *heap_end;
    char tx_str[20];
    mrf_evt evt;
    // Initialize all modules
    uart_init(115200);
    accel_init();
    touch_init((1 << 9) | (1 << 10));       // Channels 9 and 10
    // usb_init();
    setvbuf(stdin, NULL, _IONBF, 0);        // No buffering

    // Run tests
    tests();
    delay(500);
    spi_init(0,0,0);
    //gpio_input_enable(FRDM_GPIO_PORT_D, 0, FRDM_IRQC_FALLING_EDGE);
    //RGB_LED(0,100,0);                       // Green


   
    mrf24j40_init();
    mrf24j40_devinit();
    delay(100);
    mrf24j40_setpan(0xcafe);
    mrf24j40_setShortAddr(0x6000);
    for(;;) {
        //sprintf(tx_str, "%d %d", touch_data(9), touch_data(10));
        sprintf(tx_str, "%d,%d,%d,", accel_x(), accel_y(), accel_z()); 
        iprintf("%s ", tx_str);

        delay(30);
        mrf24j40_interrupt_handler();
        evt = mrf24j40_check_flags();
        if (evt & mrf_rxevent) {
            iprintf("mrf_rxevent\r\n");
        }
        if (evt & mrf_txevent) {
            iprintf("mrf_txevent\r\n");
        }
        mrf24j40_send16(0x6001, tx_str);
    }
}

