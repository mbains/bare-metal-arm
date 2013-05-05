/* 
 * File:   spi.h
 * Author: mbains
 *
 * Created on April 23, 2013, 7:08 PM
 */

#ifndef BAREMETAL_SPI_H
#define	BAREMETAL_SPI_H
#include <freedom.h>
#include "common.h"

void spi_init(int cpol, int cphase, int rate);

//uint8_t spi_read_data(void);

void spi_write_data(uint8_t byteIn);

void SPI0_IRQHandler() __attribute__((interrupt("IRQ")));
uint8_t getSPIData();
#endif	/* SPI_H */

