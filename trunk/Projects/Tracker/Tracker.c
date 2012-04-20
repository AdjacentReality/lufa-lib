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
#include "twi.h"
#include "l3g.h"
#include "lsm303.h"
#include "MadgwickAHRS.h"
#include "packet.h"

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

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

    twi_init();
	
	/* Hardware Initialization */
	Buttons_Init();
	LEDs_Init();
	USB_Init();
	Serial_Init(38400, false);
	
	// F_CPU/1024
	TCCR1B = 0x05;
	
	// Set the battery charge current to 500mA
	DDRB |= (1 << 7);
	PORTB |= (1 << 7);
}

static void SetupSensors(void)
{
	l3g_init();
	lsm303_init();
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	sei();

    SetupSensors();

    // Reset the timer so the setup time isn't taken into account
    TCNT1 = 0;
	for (;;)
	{
		CheckSensors();

		/* Must throw away unused bytes from the host, or it will lock up while waiting for the device */
		CDC_Device_ReceiveByte(&Tracker_CDC_Interface);

		CDC_Device_USBTask(&Tracker_CDC_Interface);
		USB_USBTask();
	}
}

static short g[3], m[3], a[3];
static float gf[3];

#define GYRO_TO_RADIANS(a) ((float)a*M_PI*L3G_SO/(180.0*1000.0))

/** Reads the values of the sensors and prints out to USB. */
void CheckSensors(void)
{
    packet_t p;
    p.type = PACKET_QUAT;

    l3g_read(&g[0], &g[1], &g[2]);
    lsm303_m_read(&m[0], &m[1], &m[2]);
    lsm303_a_read(&a[0], &a[1], &a[2]);
    
    gf[0] = GYRO_TO_RADIANS(g[0]);
    gf[1] = GYRO_TO_RADIANS(g[1]);
    gf[2] = GYRO_TO_RADIANS(g[2]);
    
    // to avoid the frequency measurement from impacting itself
    int elapsed = TCNT1;
    TCNT1 = 0;
    float freq = ((float)F_CPU)/(1024.0*(float)(elapsed));
    
    // the acc values get normalized inside, so we should be ok not scaling
    // mag is scaled because x/y has a different sensitivity than z
    MadgwickAHRSupdate(freq, gf[0], gf[1], gf[2], (float)a[0], (float)a[1], (float)a[2],
                        (float)m[0]/1100.0, (float)m[1]/1100.0, (float)m[2]/980.0);

    p.data.quat[0] = q0;
    p.data.quat[1] = q1;
    p.data.quat[2] = q2;
    p.data.quat[3] = q3;

//    unsigned char tmp[64];
//    sprintf(tmp, "%d %lX %.2f %.2f %.2f %.2f\n", packed_size, *(uint32_t *)buf, q0, q1, q2, q3);
//    CDC_Device_SendString(&Tracker_CDC_Interface, tmp);

    unsigned char buf[PACKET_MAX_SIZE];
    int packed_size = packet_pack(&p, buf);
//    Serial_SendData(buf, packed_size);
    CDC_Device_SendData(&Tracker_CDC_Interface, buf, packed_size);
    
    // USB uses 64 byte chunks. if we don't flush, it'll wait up to 1ms to send
    CDC_Device_Flush(&Tracker_CDC_Interface);
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&Tracker_CDC_Interface);

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&Tracker_CDC_Interface);
}

