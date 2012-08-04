/*
             LUFA Library
     Copyright (C) Dean Camera, 2012.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2012  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the Tracker tester.
 */

#include "Tracker.h"
#include "version.h"
#include "led.h"
#include "twi.h"
#include "l3g.h"
#include "lsm303.h"
#include "MadgwickAHRS.h"
#include "packet.h"
#include "uart.h"
#include "calibration.h"
#include "gpio.h"

/** LED mask for the library LED driver, to indicate that the USB interface is not ready. */
static const unsigned char LEDMASK_USB_NOTREADY[3] = {255, 0, 0};

/** LED mask for the library LED driver, to indicate that the USB interface is enumerating. */
static const unsigned char LEDMASK_USB_ENUMERATING[3] = {255, 255, 0};

/** LED mask for the library LED driver, to indicate that the USB interface is ready. */
static const unsigned char LEDMASK_USB_READY[3] = {0, 255, 0};

/** LED mask for the library LED driver, to indicate that an error has occurred in the USB interface. */
static const unsigned char LEDMASK_USB_ERROR[3] = {255, 0, 0};

// Default to streaming just quaternions
static uint8_t streaming_mode = (1 << PACKET_QUAT);

#define SHOULD_STREAM(m) (streaming_mode & (1 << m))

static bool demo_mode = true;

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t Tracker_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = 0,

				.DataINEndpointNumber           = CDC_TX_EPNUM,
				.DataINEndpointSize             = CDC_TXRX_EPSIZE,
				.DataINEndpointDoubleBank       = false,

				.DataOUTEndpointNumber          = CDC_RX_EPNUM,
				.DataOUTEndpointSize            = CDC_TXRX_EPSIZE,
				.DataOUTEndpointDoubleBank      = false,

				.NotificationEndpointNumber     = CDC_NOTIFICATION_EPNUM,
				.NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
				.NotificationEndpointDoubleBank = false,
			},
	};

bool useUSB = 0;

static inline void SetPower(bool power)
{
#if TRACKER_BOARD_REVISION == 2
	// Enable or disable buck regulator 1
	PORTB = (PORTB & ~(1 << 6)) | (power << 6);
#elif TRACKER_BOARD_REVISION == 3
    PORTD = (PORTD & ~(1 << 5)) | (power << 5);
#endif /* TRACKER_BOARD_REVISION */
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
	
	/* Hardware Initialization */
	twi_init();
//	led_init();
//	gpio_init();
	USB_Init();
	uart_init(38400, false);
	
	// Set the 16 bit timer to increment at F_CPU/1024 Hz
	TCCR1B = 0x05;

#if TRACKER_BOARD_REVISION == 2
    // Setup LTC3554 charger/regulator
	// Set the battery charge current to 500mA
	DDRB |= (1 << 7);
	PORTB |= (1 << 7);

	// Set output pin for buck regulator 1
	DDRB |= (1 << 6);
#elif TRACKER_BOARD_REVISION == 3
    // Setup LTC3553 charger/regulator
    // Set output pin for standby and buck regulator
    DDRD |= (1 << 5) | (1 << 6);
    // For now, disable standby
    PORTD &= ~(1 << 6);
#endif /* TRACKER_BOARD_REVISION */
    
	SetPower(1);
}

static void SetupSensors(void)
{
    calibration_load();

	l3g_init();
	lsm303_init();
}

static void UpdateDemo(void)
{
    float qq2 = q2*q2;
    float roll = atan2f(2.0*(q0*q1+q2*q3), 1.0-2.0*(q1*q1+qq2));
    float rotation = atan2f(2.0*(q0*q3+q1*q2), 1.0-2.0*(qq2+q3*q3));
    
    // HSV to RGB from http://www.cs.rit.edu/~ncs/color/t_convert.html
    float h = (rotation+M_PI)/(M_PI/3.0);
    float v = 255.0*(1.0-fabsf(roll)/M_PI);
    int i = floorf(h);
    float f = h - i;
    uint8_t q = v*(1.0 - f);
    uint8_t t = v*(1.0 - (1.0 - f));
    uint8_t r, g, b;
    
    switch (i) {
        case 0:
            r = v;
            g = t;
            b = 0;
            break;
        case 1:
            r = q;
            g = v;
            b = 0;
            break;
        case 2:
            r = 0;
            g = v;
            b = t;
            break;
        case 3:
            r = 0;
            g = q;
            b = v;
            break;
        case 4:
            r = t;
            g = 0;
            b = v;
            break;
        default:
            r = v;
            g = 0;
            b = q;
            break;
    }
    
    if (b < 64 && r < 192) b = 0;
    led_set_colors(r, g, b);
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	led_set_array(LEDMASK_USB_NOTREADY);
	sei();

    SetupSensors();

    // Reset the timer so the setup time isn't taken into account
    TCNT1 = 0;
	for (;;)
	{
	    ReadData();
		CheckSensors();
		SendData();
        
        if (demo_mode) UpdateDemo();
        
        /* Must throw away unused bytes from the host, or it will lock up while waiting for the device */
        if (!useUSB) // we otherwise grab bytes in ReadData if we are using USB
    		CDC_Device_ReceiveByte(&Tracker_CDC_Interface);

		CDC_Device_USBTask(&Tracker_CDC_Interface);
		USB_USBTask();
	}
}

static void ParseByte(unsigned char c)
{
    packet_t p;
    // use the packets as they complete
    if (packet_unpack(&p, c)) {
        // exit demo mode if we get a packet
        demo_mode = false;
        switch(p.type) {
            // TODO: the rest of the packet types
        
            case PACKET_COLOR:
                led_set_array(p.data.color);
                break;
                
            case PACKET_STREAM:
                streaming_mode = p.data.bitmask;
                break;
                
            case PACKET_CAL:
                calibration_store(p.data.calibration[0], p.data.calibration[1],
                    p.data.calibration[2], p.data.calibration[3],
                    p.data.calibration[4], p.data.calibration[5]);
                break;
                
            case PACKET_GPIO_DDR:
                gpio_set_ddr(p.data.bitmask);
                break;
                
            case PACKET_GPIO_PORT:
                gpio_set_port(p.data.bitmask);
                break;
                
            case PACKET_POWER:
                SetPower(p.data.bitmask != 0);
                break;
        }
    }
}

void ReadData(void)
{
    if (useUSB) {
        int to_read = CDC_Device_BytesReceived(&Tracker_CDC_Interface);
        if (to_read > 0) {
            while (to_read--) {
                int16_t in_byte = CDC_Device_ReceiveByte(&Tracker_CDC_Interface);
                if (in_byte >= 0)
                    ParseByte(in_byte);
                else
                    break; // stop if we fail to read a byte
            }
        }
    } else {
        while (1) {
            int16_t in_byte = uart_getc();
            if (in_byte >= 0)
                ParseByte(in_byte);
            else
                break;
        }
    }
}

#define GYRO_TO_RADIANS(a) ((float)a*M_PI*L3G_SO/(180.0*1000.0))

short g[3], m[3], a[3];
float gf[3], mf[3];

/** Reads the values of the sensors and prints out to USB. */
void CheckSensors(void)
{
    l3g_read(&g[0], &g[1], &g[2]);
    lsm303_m_read(&m[0], &m[1], &m[2]);
    lsm303_a_read(&a[0], &a[1], &a[2]);
    
    // avoid the massive cpu cost if we aren't actually sending quaternions
    if (SHOULD_STREAM(PACKET_QUAT)) {
        gf[0] = GYRO_TO_RADIANS(g[0]);
        gf[1] = GYRO_TO_RADIANS(g[1]);
        gf[2] = GYRO_TO_RADIANS(g[2]);
        
        // offset and scale away some of the hard iron and soft iron error
        calibration_apply(m[0], &mf[0], m[1], &mf[1], m[2], &mf[2]);

        // to avoid the frequency measurement from impacting itself
        int elapsed = TCNT1;
        TCNT1 = 0;
        float freq = ((float)F_CPU)/(1024.0*(float)(elapsed));

        // the acc values get normalized inside, so we should be ok not scaling
        // mag is scaled because x/y has a different sensitivity than z
        MadgwickAHRSupdate(freq, gf[0], gf[1], gf[2], (float)a[0], (float)a[1], (float)a[2],
                            mf[0], mf[1], mf[2]);
    }
}

static void PackAndSend(packet_p p)
{
    unsigned char buf[PACKET_MAX_SIZE];
    int packed_size = packet_pack(p, buf);
    if (useUSB)
        CDC_Device_SendData(&Tracker_CDC_Interface, (const char const *)buf, packed_size);
    else
        uart_send(buf, packed_size);
}

void SendData(void)
{
    packet_t p;
    
    if (SHOULD_STREAM(PACKET_QUAT)) {
        p.type = PACKET_QUAT;
        p.data.quat[0] = q0;
        p.data.quat[1] = q1;
        p.data.quat[2] = q2;
        p.data.quat[3] = q3;
        PackAndSend(&p);
    }
    
    if (SHOULD_STREAM(PACKET_ACC)) {
        p.type = PACKET_ACC;
        p.data.sensor[0] = a[0];
        p.data.sensor[1] = a[1];
        p.data.sensor[2] = a[2];
        PackAndSend(&p);
    }
    
    if (SHOULD_STREAM(PACKET_GYRO)) {
        p.type = PACKET_GYRO;
        p.data.sensor[0] = g[0];
        p.data.sensor[1] = g[1];
        p.data.sensor[2] = g[2];
        PackAndSend(&p);
    }
    
    if (SHOULD_STREAM(PACKET_MAG)) {
        p.type = PACKET_MAG;
        p.data.sensor[0] = m[0];
        p.data.sensor[1] = m[1];
        p.data.sensor[2] = m[2];
        PackAndSend(&p);
    }
    
    // TODO: temperature support
    
    if (SHOULD_STREAM(PACKET_GPIO)) {
        p.type = PACKET_GPIO;
        p.data.bitmask = gpio_pin();
        PackAndSend(&p);
    }
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	led_set_array(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	led_set_array(LEDMASK_USB_NOTREADY);
	useUSB = 0;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&Tracker_CDC_Interface);

	led_set_array(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
	useUSB = ConfigSuccess;
    if (ConfigSuccess) demo_mode = false;
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&Tracker_CDC_Interface);
}

