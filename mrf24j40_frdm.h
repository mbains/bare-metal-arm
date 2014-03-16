/* 
 * File:   mrf24j40_frdm.h
 * Author: mbains
 *
 * Created on March 15, 2014, 1:26 PM
 */

#ifndef MRF24J40_FRDM_H
#define	MRF24J40_FRDM_H


#include "spi.h"

typedef enum {
    mrf_noevent = 0,
    mrf_rxevent,
    mrf_txevent
}mrf_evt;

typedef struct _rx_info_t {
    uint8_t frame_length;
    uint8_t rx_data[116]; //max data length = (127 aMaxPHYPacketSize - 2 Frame control - 1 sequence number - 2 panid - 2 shortAddr Destination - 2 shortAddr Source - 2 FCS)
    uint8_t lqi;
    uint8_t rssi;
} rx_info_t;

/**
 * Based on the TXSTAT register, but "better"
 */
typedef struct _tx_info_t {
    uint8_t tx_ok : 1;
    uint8_t retries : 2;
    uint8_t channel_busy : 1;
} tx_info_t;



void mrf24j40_init();
uint8_t mrf24j40_getAckTMOut();
void mrf24j40_devinit();
void mrf24j40_setpan(uint16_t pan_id);
void mrf24j40_setShortAddr(uint16_t address16);
void mrf24j40_send16(uint16_t dest16, char * data);
uint16_t mrf24j40_getpan();
uint16_t mrf24j40_getShortAddr();
mrf_evt mrf24j40_check_flags();
void mrf24j40_interrupt_handler(void);
rx_info_t * mrf24j40_getRxInfo();
uint16_t mrf24j40_getRxDataLength();


#endif	/* MRF24J40_FRDM_H */

