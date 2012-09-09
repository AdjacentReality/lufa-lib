#ifndef _NRF24L01P_H_
#define _NRF24L01P_H_

// Instructions
#define R_REGISTER          0x00
#define W_REGISTER          0x20
#define R_RX_PAYLOAD        0x61
#define W_TX_PAYLOAD        0xA0
#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define REUSE_TX_PL         0xE3
#define R_RX_PL_WID         0x60
#define W_ACK_PAYLOAD       0xA8
#define W_TX_PAYLOAD_NOACK  0xB0
#define NOP                 0xFF

// Registers
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define RPD         0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

// Bit shifts
// CONFIG
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0

// EN_AA
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0

// EN_RXADDR
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0

// SETUP_AW
#define AW          0

// SETUP_RETR
#define ARD         4
#define ARC         0

// RF_SETUP
#define CONT_WAVE   7
#define RF_DR_LOW   5
#define PLL_LOCK    4
#define RF_DR_HIGH  3
#define RF_PWR      1

// STATUS
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0

// OBSERVE_TX
#define PLOS_CNT    4
#define ARC_CNT     0

// FIFO_STATUS
#define TX_REUSE        6
#define TX_FULL_FIFO    5
#define TX_EMPTY        4
#define RX_FULL         1
#define RX_EMPTY        0

// DYNPD
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0

// FEATURE
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0


// Parameters
// SETUP_AW
#define AW_3        1
#define AW_4        2
#define AW_5        3

// SETUP_RETR
#define ARD_DELAY(u)    ((u/250)-1) // 250 to 4000 uS

// RF_SETUP
#define RF_PWR_18   0
#define RF_PWR_12   1
#define RF_PWR_6    2
#define RF_PWR_0    3

// STATUS
#define RX_P_NO_GET(b)  ((b >> RX_P_NO) & 0x03)
#define RX_P_NO_NU      0x06
#define RX_P_NO_EMPTY   0x07

// OBSERVE_TX
#define PLOS_CNT_GET(b) ((b >> PLOS_CNT) & 0x0F)
#define ARC_CNT_GET(b)  ((b >> ARC_CNT) & 0x0F)

#endif /* _NRF24L01P_H_ */
