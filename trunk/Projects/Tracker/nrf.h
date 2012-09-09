#ifndef _NRF_H_
#define _NRF_H_

#include <stdbool.h>

typedef struct nrf_status_struct {
    unsigned int tx_full:1;
    unsigned int rx_pipe:3;
    unsigned int max_rt:1;
    unsigned int tx_sent:1;
    unsigned int rx_ready:1;
} nrf_status_t;

void nrf_init(bool isReceiver);
bool nrf_rx_available(void);
bool nrf_tx_available(void);
nrf_status_t nrf_statistics(unsigned char *retransmits, unsigned char *lost, unsigned char *detect);
// Reset a timed out packet
void nrf_tx_reset(void);
void nrf_tx_flush(void);
void nrf_send_payload(unsigned char *payload, unsigned char len);
void nrf_read_payload(unsigned char *payload, unsigned char len);
void nrf_close(void);

#endif /* _NRF_H_ */
