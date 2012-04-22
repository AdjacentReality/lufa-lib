#ifndef _LED_H_
#define _LED_H_

void led_init(void);
void led_set_array(const unsigned char *rgb);
void led_set_colors(unsigned char red, unsigned char green, unsigned char blue);
void led_set_ir(unsigned char ir);

#endif /* _LED_H_ */

