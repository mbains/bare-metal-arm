//
// demo.c -- Simple demonstration program
//
//  Copyright (c) 2012-2013 Andrew Payne <andy@payne.org>
//

#include <stdio.h>
#include "freedom.h"
#include "common.h"
#include "gpio.h"

extern char *_sbrk(int len);

#define MK_DISCRETE(x, y) ((( x<500 ? 0: x)/4000.)*y)
// Main program
int main(void)
{
    char i;
    char *heap_end;
    int red, green, blue;
    int touchSig;
    float touch, touchlast, touchdiff;
    float brightness;
    
    // Initialize all modules
    uart_init(115200);
    accel_init();
    touch_init((1 << 9) | (1 << 10));       // Channels 9 and 10
    // usb_init();
    setvbuf(stdin, NULL, _IONBF, 0);        // No buffering

    // Run tests
    tests();
    delay(500);
    //gpio_input_enable(FRDM_GPIO_PORT_D, 0, FRDM_IRQC_FALLING_EDGE);
    RGB_LED(0,100,0);                       // Green

    // Welcome banner
    iprintf("\r\n\r\n====== Freescale Freedom FRDM-LK25Z\r\n");
    iprintf("Built: %s %s\r\n\r\n", __DATE__, __TIME__);
    heap_end = _sbrk(0);
    iprintf("Heap:  %p to %p (%d bytes used)\r\n", __heap_start, heap_end, 
                heap_end - (char *)__heap_start);
    iprintf("Stack: %p to %p (%d bytes used)\r\n", &i, __StackTop, 
                (char *)__StackTop - &i);
    iprintf("%d bytes free\r\n", &i - heap_end);
    brightness = 100;
    touchlast = 0;
    touchSig = 0;
    for(;;) {
        
        delay(10);
        touch = touch_data(10);
        touchdiff = touch - touchlast;
        if( ((touchdiff > 10) || (touchdiff < -10)) && !(touchdiff < -100))
        {
            touchSig += 1;
            touchlast = touch;
        }
       
        if( (touch > 30) & (touchSig > 1)) {
            touchSig = 0;
            brightness = (touch/750.)*255;
            brightness = (( brightness>255)? 255: brightness);
        }
        red = accel_x();
        green = accel_y();
        blue = accel_z();
        
        red = MK_DISCRETE(red, brightness);
        green = MK_DISCRETE(green, brightness);
        blue = MK_DISCRETE(blue, brightness);
        RGB_LED(red, green, blue);
        
        
//        iprintf("monitor> ");
//        
//        iprintf("\r\n");
//        iprintf("Inputs:  x=%5d   y=%5d   z=%5d ", accel_x(), accel_y(), accel_z());
//        iprintf("touch=(%d,%d)\r\n", touch_data(9), touch_data(10));
        //iprintf("gpio IRQ count:(%d)\r\n", (int)getPortD_IRQ_count());
        // usb_dump();
    }
}
