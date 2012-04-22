#include "led.h"
#include <avr/io.h>

#define LED_RED PORTD7
#define LED_GREEN PORTC6
#define LED_BLUE PORTD6
#define LED_IR PORTC7

void led_init(void)
{
    DDRC |= (1 << LED_GREEN) | (1 << LED_IR);
    // we're sinking all LEDs into the pins, so set them high to turn off
    PORTC |= (1 << LED_GREEN) | (1 << LED_IR);
    DDRD |= (1 << LED_RED) | (1 << LED_BLUE);
    PORTD |= (1 << LED_RED) | (1 << LED_BLUE);
}

void led_set_array(const unsigned char *rgb)
{
    led_set_colors(rgb[0], rgb[1], rgb[2]);
}

void led_set_colors(unsigned char red, unsigned char green, unsigned char blue)
{
    if (red != 0) PORTD &= ~(1 << LED_RED);
    else PORTD |= (1 << LED_RED);

    if (green != 0) PORTC &= ~(1 << LED_GREEN);
    else PORTC |= (1 << LED_GREEN);
    
    if (blue != 0) PORTD &= ~(1 << LED_BLUE);
    else PORTD |= (1 << LED_BLUE);
}

void led_set_ir(unsigned char ir)
{
    if (ir != 0) PORTC &= ~(1 << LED_IR);
    else PORTC |= (1 << LED_IR);
}

