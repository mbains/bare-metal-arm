/* 
 * File:   gpio.h
 * Author: Manjinder Bains
 *
 * Created on April 6, 2013, 12:21 AM
 */

#ifndef BAREMETAL_GPIO_H
#define	BAREMETAL_GPIO_H
#include <freedom.h>
#include "common.h"

#define FRDM_IRQC_RISING_EDGE   0b1001
#define FRDM_IRQC_FALLING_EDGE  0b1010
#define FRDM_MUX_GPIO           1u
         
#define FRDM_GPIO_PIN(x)        ((uint32_t)(1 << (uint32_t)(x)))

#define FRDM_PORTD_SET_PIN(x) ((PTD_BASE_PTR->PDOR |= (1<<(x))))
#define FRDM_PORTD_CLEAR_PIN(x) ((PTD_BASE_PTR->PDOR &= ~(1<<(x))))

typedef enum {
    FRDM_GPIO_PORT_A,
    FRDM_GPIO_PORT_B,
    FRDM_GPIO_PORT_C,
    FRDM_GPIO_PORT_D
}FRDM_GPIO_PORT_T;


void gpio_init(void);
void gpio_input_enable(FRDM_GPIO_PORT_T port, uint8_t pin, uint8_t irqc);
void gpio_output_enable(FRDM_GPIO_PORT_T port, uint8_t pin);

uint32_t getPortD_IRQ_count(void);
void PORTD_IRQHandler() __attribute__((interrupt("IRQ")));
void PORTA_IRQHandler() __attribute__((interrupt("IRQ")));

#endif	/* GPIO_H */

