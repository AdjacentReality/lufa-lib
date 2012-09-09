#ifndef _NRF_H_
#define _NRF_H_

#include <stdbool.h>

void nrf_init(void);

bool nrf_rx_available(void);

bool nrf_tx_available(void);

void nrf_close(void);

#endif /* _NRF_H_ */
