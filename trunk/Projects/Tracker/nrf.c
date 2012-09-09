#include "nrf.h"
#include "nrf24l01p.h"
#include <avr/io.h>
#include <stdlib.h>
#include <LUFA/Drivers/Peripheral/SPI.h>

#define CE  PORTB5
#define CSN PORTB0

#define CE_LOW()    PORTB &= ~(1 << CE)
#define CE_HIGH()   PORTB |= (1 << CE)
#define CSN_LOW()   PORTB &= ~(1 << CSN)
#define CSN_HIGH()  PORTB |= (1 << CSN)

static unsigned char nrf_write(unsigned char reg, unsigned char value)
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

void nrf_init(void)
{
    // Set the necessary pins as outputs;
    DDRB |= (1 << CSN) | (1 << PORTB1) | (1 << PORTB2) | (1 << CE);
    CE_LOW();
    CSN_HIGH();
    
    // TODO: set PRX or PTX
    // TODO: determine safe retransmission times and counts
    // TODO: use a non-default channel?
    // TODO: set addresses (on PTX, TX and RX0 should be equal)
    // TODO: enable dynamic payload and acking payload
    
    nrf_write(W_REGISTER | SETUP_AW, (AW_3 << AW)); // use 3 byte addresses
    nrf_write(W_REGISTER | RF_SETUP, (1 << RF_DR_HIGH) | (RF_PWR_0 << RF_PWR)); // use 2Mbps and 0dBm
    nrf_write(W_REGISTER | FEATURE, (1 << EN_DYN_ACK)); // allow non-acked transmits
    nrf_write(W_REGISTER | CONFIG, (1 << EN_CRC) | (1 << PWR_UP)); // turn on!
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

void nrf_close(void)
{
    CE_LOW();
    nrf_write(W_REGISTER | CONFIG, (1 << EN_CRC)); // set PWR_UP low
}

