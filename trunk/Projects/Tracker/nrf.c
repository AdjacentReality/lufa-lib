#include "nrf.h"
#include "nrf24l01p.h"
#include <avr/io.h>
#include <stdlib.h>
#include <LUFA/Drivers/Peripheral/SPI.h>

#if TRACKER_BOARD_REVISION == 3
#define CE          PORTE6
#define CE_LOW()    PORTE &= ~(1 << CE)
#define CE_HIGH()   PORTE |= (1 << CE)
#elif TRACKER_BASE_STATION
#define CE          PORTB5
#define CE_LOW()    PORTB &= ~(1 << CE)
#define CE_HIGH()   PORTB |= (1 << CE)
#endif /* TRACKER_BOARD_REVISION */

#define CSN         PORTB0
#define CSN_LOW()   PORTB &= ~(1 << CSN)
#define CSN_HIGH()  PORTB |= (1 << CSN)

#define ADDRESS_P0  0xE7E7E7E7

typedef union statusConverter {
    nrf_status_t status;
    unsigned char raw;
} statusConverter;

static unsigned char nrf_write(unsigned char reg)
{
    CSN_LOW();
    unsigned char status = SPI_TransferByte(reg);
    CSN_HIGH();
    
    return status;
}

static unsigned char nrf_write_value(unsigned char reg, unsigned char value)
{
    CSN_LOW();
    unsigned char status = SPI_TransferByte(reg);
    SPI_SendByte(value);
    CSN_HIGH();
    
    return status;
}

static unsigned char nrf_read(unsigned char reg, unsigned char *value)
{
    CSN_LOW();
    unsigned char status = SPI_TransferByte(reg);
    *value = SPI_ReceiveByte();
    CSN_HIGH();
    
    return status;
}

static unsigned char nrf_write_array(unsigned char reg, unsigned char *array, unsigned char len)
{
    CSN_LOW();
    unsigned char status = SPI_TransferByte(reg);
    unsigned char i;
    for (i = 0; i < len; i++)
        SPI_SendByte(array[i]);
    CSN_HIGH();
    
    return status;
}

static unsigned char nrf_read_array(unsigned char reg, unsigned char *array, unsigned char len)
{
    CSN_LOW();
    unsigned char status = SPI_TransferByte(reg);
    unsigned char i;
    for (i = 0; i < len; i++)
        array[i] = SPI_ReceiveByte();
    CSN_HIGH();
    
    return status;
}

void nrf_init(bool isReceiver)
{
    // Set the necessary pins as outputs;
#if TRACKER_BOARD_REVISION == 3
    DDRB |= (1 << CSN) | (1 << PORTB1) | (1 << PORTB2);
    DDRE |= (1 << CE);
#elif TRACKER_BASE_STATION
    DDRB |= (1 << CSN) | (1 << PORTB1) | (1 << PORTB2) | (1 << CE);
#endif /* TRACKER_BOARD_REVISION */
    CE_LOW();
    CSN_HIGH();
    
    // TODO: determine safe retransmission times and counts
    // TODO: use a non-default channel?
    // TODO: enable dynamic payload and acking payload
    
    nrf_write_value(W_REGISTER | SETUP_AW, (AW_3 << AW)); // use 3 byte addresses
    nrf_write_value(W_REGISTER | RF_SETUP, (1 << RF_DR_HIGH) | (RF_PWR_0 << RF_PWR)); // use 2Mbps and 0dBm
    nrf_write_value(W_REGISTER | FEATURE, (1 << EN_DYN_ACK)); // allow non-acked transmits

    // TODO: set the addresses dynamically to allow multiple transmitters
/*
    uint32_t addr = ADDRESS_P0;
    if (isReceiver) {
        // note that avr-libc and the nrf both operate little endian
        nrf_write_array(W_REGISTER | RX_ADDR_P0, (unsigned char *)&addr);
    } else {
        nrf_write_array(W_REGISTER | TX_ADDR, (unsigned char *)&addr);
        // set the rx address the same as tx for ack to work
        nrf_write_array(W_REGISTER | RX_ADDR_P0, (unsigned char *)&addr);
    }
*/
    
    // turn on as transmitter or receiver
    nrf_write_value(W_REGISTER | CONFIG, (isReceiver << PRIM_RX) | (1 << EN_CRC) | (1 << PWR_UP));
    
    // TODO: save 300uA by going high only when actually transmitting
    CE_HIGH();
}

bool nrf_rx_available(void)
{
    unsigned char status;
    nrf_read(R_REGISTER | FIFO_STATUS, &status);
    
    return !((status >> RX_EMPTY) & 1);
}
    
bool nrf_tx_available(void)
{
    unsigned char status;
    nrf_read(R_REGISTER | FIFO_STATUS, &status);
    
    return !(status >> TX_FULL_FIFO) & 1;
}

nrf_status_t nrf_statistics(unsigned char *retransmits, unsigned char *lost, unsigned char *detect)
{
    unsigned char stats;
    statusConverter status;
    
    nrf_read(R_REGISTER | RPD, detect);
    
    status.raw = nrf_read(R_REGISTER | OBSERVE_TX, &stats);
    *retransmits = ARC_CNT_GET(stats);
    *lost = PLOS_CNT_GET(stats);
    
    return status.status;
}

void nrf_tx_reset(void)
{
    nrf_write_value(W_REGISTER | STATUS, (1 << MAX_RT) | (1 << TX_DS));
}

void nrf_tx_flush(void)
{
    nrf_write(FLUSH_TX);
}

void nrf_send_payload(unsigned char *payload, unsigned char len)
{
    nrf_write_array(W_TX_PAYLOAD, payload, len);
}

void nrf_read_payload(unsigned char *payload, unsigned char len)
{
    // TODO: encapsulate this, including a header stating which endpoint it is from
    nrf_read_array(R_RX_PAYLOAD, payload, len);
}

void nrf_close(void)
{
    CE_LOW();
    nrf_write_value(W_REGISTER | CONFIG, (1 << EN_CRC)); // set PWR_UP low
}

