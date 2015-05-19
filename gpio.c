/* 
 * File:   gpio.c
 * Author: Manjinder Bains
 *
 * Created on April 6, 2013, 12:21 AM
 * TODO: Implement output features
 */


#include "gpio.h"
#include "common.h"
#include <stddef.h>


volatile static int totalPortD_irq;
volatile static int totalPortA_irq;

typedef struct {
    volatile uint32_t * pcr_ptr;
    volatile uint32_t * pddr_ptr;
} CtrlDirRegisters_t;

CtrlDirRegisters_t get_ctrl_dir_registers(FRDM_GPIO_PORT_T port, uint8_t pin)
{
    CtrlDirRegisters_t reg;

    switch(port) {
        case FRDM_GPIO_PORT_A:
            reg.pcr_ptr = &(PORT_PCR_REG(PORTA_BASE_PTR, pin));
            reg.pddr_ptr = &(GPIOA_PDDR);
            break;
        case FRDM_GPIO_PORT_B:
            reg.pcr_ptr = &(PORT_PCR_REG(PORTB_BASE_PTR, pin));
            reg.pddr_ptr = &(GPIOB_PDDR);
            break;
        case FRDM_GPIO_PORT_C:
            reg.pcr_ptr = &(PORT_PCR_REG(PORTC_BASE_PTR, pin));
            reg.pddr_ptr = &(GPIOC_PDDR);
            break;
        case FRDM_GPIO_PORT_D:
            reg.pcr_ptr = &(PORT_PCR_REG(PORTD_BASE_PTR, pin));
            reg.pddr_ptr = &(GPIOD_PDDR);
            break;
    }
    
    return reg;
}

void gpio_input_enable(FRDM_GPIO_PORT_T port, uint8_t pin, uint8_t irqc) 
{
    
    CtrlDirRegisters_t cdreg = get_ctrl_dir_registers(port, pin);
    IRQInterruptIndex irq = -1;
    switch(port) {
        case FRDM_GPIO_PORT_A:
            totalPortA_irq = 0;
            irq = INT_PORTA;
            break;
        case FRDM_GPIO_PORT_B:
            break;
        case FRDM_GPIO_PORT_C:
            break;
        case FRDM_GPIO_PORT_D:
            totalPortD_irq = 0;
            irq = INT_PORTD;
            break;               
    }
    
    //set the PCR MUX to GPIO alternative 1
    *(cdreg.pcr_ptr) |= PORT_PCR_MUX(FRDM_MUX_GPIO); 
    //set the direction register for input for this pin
    *(cdreg.pddr_ptr) &= ~GPIO_PDDR_PDD(1 << pin); 
    
    
    //enable interrupts on pin if possible
    if(irq != -1) {
        *(cdreg.pcr_ptr) = CMN_SET_CLR_BITS(*(cdreg.pcr_ptr), PORT_PCR_IRQC(irqc), PORT_PCR_IRQC(0xF));//set irqc, clear others
        enable_irq(irq);
    }
    
}

void gpio_output_enable(FRDM_GPIO_PORT_T port, uint8_t pin)
{
    CtrlDirRegisters_t cdreg = get_ctrl_dir_registers(port, pin);
    *(cdreg.pcr_ptr) = PORT_PCR_MUX(FRDM_MUX_GPIO); //select GPIO
    *(cdreg.pddr_ptr) |= GPIO_PDDR_PDD(1 << pin); //set direction output
}

inline uint32_t getPortD_IRQ_count(void) {
    return totalPortD_irq;
}

void PORTD_IRQHandler() {
    totalPortD_irq+=1;
    //iprintf("PD5:%d\r\n",totalPortD_irq);
    //set the interrupt status flag to complete IRQ handle
    //TODO this handles only portd pin 5
    PORTD_PCR5 |= (1 << PORT_PCR_ISF_SHIFT);

}
inline uint32_t getPortA_IRQ_count(void) {
    return totalPortA_irq;
}

void PORTA_IRQHandler() {
    totalPortA_irq+=1;

    //set the interrupt status flag to complete IRQ handle
    PORTA_PCR0 |= (1 << PORT_PCR_ISF_SHIFT);
}

