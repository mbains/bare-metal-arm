/* 
 * File:   gpio.c
 * Author: Manjinder Bains
 *
 * Created on April 6, 2013, 12:21 AM
 * TODO: Implement output features
 */


#include "gpio.h"
#include <stddef.h>


volatile static int totalPortD_irq;
volatile static int totalPortA_irq;


void gpio_input_enable(FRDM_GPIO_PORT_T port, uint8_t pin, uint8_t irqc) 
{
    
    volatile uint32_t * pcr_ptr = 0;
    volatile uint32_t * pddr_ptr = 0;
    IRQInterruptIndex irq = -1;
    switch(port) {
        case FRDM_GPIO_PORT_A:
            pcr_ptr = &(PORT_PCR_REG(PORTA_BASE_PTR, pin));
            pddr_ptr = &(GPIOA_PDDR);
            totalPortA_irq = 0;
            irq = INT_PORTA;
            break;
        case FRDM_GPIO_PORT_B:
            pcr_ptr = &(PORT_PCR_REG(PORTB_BASE_PTR, pin));
            pddr_ptr = &(GPIOB_PDDR);
            break;
        case FRDM_GPIO_PORT_C:
            pcr_ptr = &(PORT_PCR_REG(PORTC_BASE_PTR, pin));
            pddr_ptr = &(GPIOC_PDDR);
            break;
        case FRDM_GPIO_PORT_D:
            pcr_ptr = &(PORT_PCR_REG(PORTD_BASE_PTR, pin));
            pddr_ptr = &(GPIOD_PDDR);
            totalPortD_irq = 0;
            irq = INT_PORTD;
            break;
                    
    }
    
    //set the PCR MUX to GPIO alternative 1
    *pcr_ptr |= PORT_PCR_MUX(FRDM_MUX_GPIO); 
    //set the direction register for input for this pin
    *pddr_ptr &= ~GPIO_PDDR_PDD(pin); 
    
    
    //enable interrupts on pin if possible
    if(irq != -1) {
        *pcr_ptr |= PORT_PCR_IRQC(irqc);
        enable_irq(irq);
    }
    
}


inline uint32_t getPortD_IRQ_count(void) {
    return totalPortD_irq;
}

void PORTD_IRQHandler() {
    totalPortD_irq+=1;

    // TODO: For some reason, interrupt keeps firing until 
    // it is disabled and re-enabled. 
    PORTD_PCR0 &= ~PORT_PCR_IRQC(FRDM_IRQC_FALLING_EDGE);
     delay(50);
    PORTD_PCR0 |= PORT_PCR_IRQC(FRDM_IRQC_FALLING_EDGE);
   
}
inline uint32_t getPortA_IRQ_count(void) {
    return totalPortA_irq;
}

void PORTA_IRQHandler() {
    totalPortA_irq+=1;

    // TODO: For some reason, interrupt keeps firing until 
    // it is disabled and re-enabled. 
    PORTA_PCR0 &= ~PORT_PCR_IRQC(FRDM_IRQC_FALLING_EDGE);
     delay(50);
    PORTA_PCR0 |= PORT_PCR_IRQC(FRDM_IRQC_FALLING_EDGE);
   
}

