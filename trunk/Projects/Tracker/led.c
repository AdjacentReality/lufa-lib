#include "led.h"
#include <avr/io.h>
#include <stdlib.h>

#if TRACKER_BOARD_REVISION == 2
#define LED_RED     PORTD7  // 4D
#define LED_GREEN   PORTC6  // 3A
#define LED_BLUE    PORTD6  // !4D
#define LED_IR      PORTC7  // 4A
#elif TRACKER_BOARD_REVISION == 3
#define LED_RED     PORTD7  // 4D
#define LED_GREEN   PORTC7  // 4A
#define LED_BLUE    PORTB6  // 4B
#define LED_IR      PORTC6  // 3A
#endif /* TRACKER_BOARD_REVISION */

void led_init(void)
{
#if TRACKER_BOARD_REVISION == 2
    // set all used pins as outputs
    DDRC |= (1 << LED_GREEN);// | (1 << LED_IR);
    // we're sinking all LEDs into the pins, so set them high to turn off
    PORTC |= (1 << LED_GREEN);// | (1 << LED_IR);
    DDRD |= (1 << LED_RED) | (1 << LED_BLUE);
    PORTD |= (1 << LED_RED) | (1 << LED_BLUE);
    
    // Timer 3 setup - clear on match PWM, clock/8
    OCR3A = 0xFF; // start green at off
    TCCR3A = (1 << COM3A1) | (1 << WGM30);
    TCCR3B = (1 << WGM32) | (1 << CS31);
    
    // Timer 4 setup
    OCR4C = 0xFF; // top at 256 counts
 //   OCR4A = 0xFF; // start IR at off
    OCR4D = 0xFF; // start red at off
    // dead time at clock/8, pwm at full clock to maximize dead time length
    TCCR4B = (1 << DTPS41) | (1 << DTPS40) | (1 << CS40);
//    TCCR4A = (1 << COM4A1) | (1 << PWM4A); // enable clear on match PWM for IR
    TCCR4C = (1 << COM4D1) | (1 << PWM4D); // enable clear on match PWM for red
#elif TRACKER_BOARD_REVISION == 3
    // set all used pins as outputs
    DDRB |= (1 << LED_BLUE);
    DDRC |= (1 << LED_GREEN) | (1 << LED_IR);
    DDRD |= (1 << LED_RED);
    // we're sinking all LEDs into the pins, so set them high to turn off
    PORTB |= (1 << LED_BLUE);
    PORTC |= (1 << LED_GREEN) | (1 << LED_IR);
    PORTD |= (1 << LED_RED);
    
    // Timer 3 setup - clear on match PWM, clock/8
//    OCR3A = 0xFF; // start IR at off
//    TCCR3A = (1 << COM3A1) | (1 << WGM30);
//    TCCR3B = (1 << WGM32) | (1 << CS31);
    
    // Timer 4 setup
    OCR4C = 0xFF; // top at 256 counts
    OCR4A = 0xFF; // start green at off
    OCR4B = 0xFF; // start blue at off
    OCR4D = 0xFF; // start red at off
    // dead time at clock/8, pwm at full clock to maximize dead time length
    TCCR4B = (1 << DTPS41) | (1 << DTPS40) | (1 << CS40);
    // enable clear on match PWM for green and blue
    TCCR4A = (1 << COM4A1) | (1 << PWM4A) | (1 << COM4B1) | (1 << PWM4B);
    // enable clear on match PWM for red
    TCCR4C = (1 << COM4D1) | (1 << PWM4D);
#endif /* TRACKER_BOARD_REVISION */
}

void led_set_array(const unsigned char *rgb)
{
    led_set_colors(rgb[0], rgb[1], rgb[2]);
}

void led_set_colors(unsigned char red, unsigned char green, unsigned char blue)
{
#if TRACKER_BOARD_REVISION == 2
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
#elif TRACKER_BOARD_REVISION == 3
    OCR4A = 0xFF - green;
    OCR4B = 0xFF - blue;
    OCR4D = 0xFF - red;
#endif /* TRACKER_BOARD_REVISION */
}

void led_set_ir(unsigned char ir)
{
#if TRACKER_BOARD_REVISION == 3
    OCR3A = 0xFF - ir;
#endif /* TRACKER_BOARD_REVISION */
}

