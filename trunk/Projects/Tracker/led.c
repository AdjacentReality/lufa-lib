#include "led.h"
#include <avr/io.h>

#define LED_RED     PORTD7  // 4D
#define LED_GREEN   PORTC6  // 3A/!4A
#define LED_BLUE    PORTD6  // !4D
#define LED_IR      PORTC7  // 4A

void led_init(void)
{
    // set all used pins as outputs
    DDRC |= (1 << LED_GREEN) | (1 << LED_IR);
    // we're sinking all LEDs into the pins, so set them high to turn off
    PORTC |= (1 << LED_GREEN) | (1 << LED_IR);
    DDRD |= (1 << LED_RED) | (1 << LED_BLUE);
    PORTD |= (1 << LED_RED) | (1 << LED_BLUE);
    
    // Timer 3 setup - clear on match PWM, clock/8
    OCR3A = 0xFF; // start green at off
    TCCR3A = (1 << COM3A1) | (1 << WGM30);
    TCCR3B = (1 << WGM32) | (1 << CS31);
    
    // Timer 4 setup
    OCR4C = 0xFF; // top at 256 counts
    OCR4A = 0xFF; // start IR at off
    OCR4D = 0xFF; // start red at off
    TCCR4B = (1 << CS42); // clock/8, but which clock is it using?
    TCCR4A = (1 << COM4A1) | (1 << PWM4A); // enable clear on match PWM for IR
    TCCR4C = (1 << COM4D1) | (1 << PWM4D); // enable clear on match PWM for red
}

void led_set_array(const unsigned char *rgb)
{
    led_set_colors(rgb[0], rgb[1], rgb[2]);
}

void led_set_colors(unsigned char red, unsigned char green, unsigned char blue)
{
    OCR4D = 0xFF - red;

    OCR3A = 0xFF - green;
    
    // I screwed up and assigned blue and red to inverted outputs of the same compare
    // TODO: fake independent PWM using the dead time generator
    if (blue != 0) PORTD &= ~(1 << LED_BLUE);
    else PORTD |= (1 << LED_BLUE);
}

void led_set_ir(unsigned char ir)
{
    OCR4A = 0xFF - ir;
}

