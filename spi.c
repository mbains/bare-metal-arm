#include <freedom.h>
#include "common.h"
#include "spi.h"
#include <stdio.h>

#define WRITE_ERR "SPIWRITE_ERR\r\n"
#define READ_ERR "SPIREAD_ERR\r\n"
#define INTRQ   "SPI IRQ\r\n"

volatile static int dataIn;

static void spiSendData(uint8_t data);
static uint8_t spiRecvData(void);

static void spiSendData(uint8_t data) 
{
    //wait while not TX Empty
    while(!(SPI0_S & SPI_S_SPTEF_MASK));
    SPI0_D = data;
}

static uint8_t spiRecvData(void)
{
    //wait while rx not full
    while(!(SPI0_S & SPI_S_SPRF_MASK));
    return SPI0_D;
}

void spi_init(int cpol, int cphase, int rate) 
{
    
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
    PORTC_PCR4 = PORT_PCR_MUX(2); //cs
    PORTC_PCR5 = PORT_PCR_MUX(2); //clk 
    PORTC_PCR6 = PORT_PCR_MUX(2); //mosi -- SDI
    PORTC_PCR7 = PORT_PCR_MUX(2); //miso -- SDO
    
    SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK; //point clock to SPI
    
    SPI0_C1 |= SPI_C1_SPE_MASK; //enable SPI subsystem
    //SPI0_C1 |= SPI_C1_SPIE_MASK; //enable recieve buffer full interrupt
    SPI0_C1 |= SPI_C1_MSTR_MASK; //master mode
    SPI0_C1 &= ~SPI_C1_CPHA_MASK; //set clock phase to 0. it defaults to 1 on reset. 
    SPI0_C1 |= SPI_C1_SSOE_MASK; //slave select enable so SS pin is automatically asserted

    //SPI0_C2 |= SPI_C2_MODFEN_MASK; //turn on MODFEN so SS pin is used for SPI slave select
    
    
    /*set the clock to max speed.
     * busclock = (core_clock/4). so bus clock = 12MHz?
     *  Doing the calculation. 
     * baud = (12E6/((sppr+1)*pow(2,(spr+1))))
       baud = 75KHz
    */
    SPI0_BR &= ~SPI_BR_SPPR_MASK;
    SPI0_BR &= ~SPI_BR_SPR_MASK;
    SPI0_BR |= SPI_BR_SPPR(4);
    SPI0_BR |= SPI_BR_SPR(1);
            
    //enable_irq(INT_SPI0);
}

void spi_write_test(uint8_t byteIn)
{
    volatile int readb; 
    //while not empty
    while(!(SPI0_S & SPI_S_SPTEF_MASK))
        ;
    //iprintf("done with first loop\r\n");
    SPI0_D = byteIn;
        

    //    //wait while not empty
    while(!(SPI0_S & SPI_S_SPTEF_MASK)) 
        ;
//    iprintf("done with second loop\r\n");
//    
      
    //wait while rx not full
    while(!(SPI0_S & SPI_S_SPRF_MASK))
        ;
    readb = SPI0_D;

    iprintf("spi_write_data: %d\r\n", readb);

    //turn on rx interrupt
    //SPI0_C1 |= SPI_C1_SPIE_MASK;
}

uint8_t spiReadWrite(uint8_t * rbuf, uint8_t * tbuf, int cnt)
{
    uint8_t i;
    for(i = 0; i < cnt; i++) {
        if(tbuf) {
            spiSendData(*tbuf++);
        } else {
            spiSendData(0xff);
        }
        if(rbuf) {
            *rbuf++ = spiRecvData();
        } else {
            //toss
            (void)spiRecvData();
        }
    }
    return i;
}


void SPI0_IRQHandler() 
{
    volatile uint8_t data = 0;

    //turn off receive interrupt
    SPI0_C1 &= ~SPI_C1_SPIE_MASK;
    if(SPI0_S & SPI_S_SPRF_MASK) 
    {
        data = SPI0_D;
        iprintf("IRQ data = %u\r\n", data);
    } else {
        uart_write_err( READ_ERR, sizeof(READ_ERR));
    }
    iprintf("IRQ DONE\r\n");    
}


// TODO:
//uint8_t spi_read_data(void)
//{
//    volatile uint8_t data =0;
//    if(SPI0_S & SPI_S_SPRF_MASK) {
//        data =  SPI0_D;
//    } else {
//        uart_write_err(READ_ERR, sizeof (READ_ERR));
//    }
//    return data;
//}