#include "led.h"
#include <avr/io.h>
#include <stdlib.h>

#define LED_RED     PORTD7  // 4D
#define LED_GREEN   PORTC6  // 3A
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
    // dead time at clock/8, pwm at full clock to maximize dead time length
    TCCR4B = (1 << DTPS41) | (1 << DTPS40) | (1 << CS40);
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
    // so try to do some fake independent PWM using the dead time generator
    int diff = (int)blue - (int)(0xFF - red);
    if ((diff > 0) && (diff <= 120)) {
        DT4 = (0x0F & (diff >> 3)); // use dead time to keep blue on for longer
        TCCR4C = (1 << COM4D0) | (1 << PWM4D); // enable inverted red and blue
    } else {
        diff = abs(diff);
        // use the regular inverted output only if it is closer to correct than 0 or 255
        if ((diff >= blue) || (diff >= (0xFF - blue))) {
            TCCR4C = (1 << COM4D1) | (1 << PWM4D); // disable inverted red and blue
            if (blue != 0) PORTD &= ~(1 << LED_BLUE);
            else PORTD |= (1 << LED_BLUE);
        } else {
            DT4 = 0; // clear dead time
            TCCR4C = (1 << COM4D0) | (1 << PWM4D); // enable inverted red and blue
        }
    }
}

void led_set_ir(unsigned char ir)
{
    OCR4A = 0xFF - ir;
}

