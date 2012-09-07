#ifndef _GPIO_H_
#define _GPIO_H_

// Disable JTAG so we can use all of PORT F
#define gpio_init() JTAG_DISABLE()

#ifdef TRACKER_BOARD_REVISION
// We use PORT F exclusively for GPIO, so we can just directly use the registers
#define gpio_set_ddr(x) (DDRF = x)
#define gpio_set_port(x) (PORTF = x)
#define gpio_pin()  (PINF)
#else // TRACKER_BASE_STATION
#define gpio_set_ddr(x) (DDRD = x)
#define gpio_set_port(x) (PORTD = x)
#define gpio_pin()  (PIND)
#endif

// TODO: support analog input

#endif /* _GPIO_H_ */
