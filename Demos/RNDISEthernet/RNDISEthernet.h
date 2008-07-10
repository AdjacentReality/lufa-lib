/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

#ifndef _RNDISETHERNET_H_
#define _RNDISETHERNET_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <string.h>

		#include "Descriptors.h"
		#include "RNDIS.h"
		#include "Ethernet.h"
		#include "TCP.h"
		#include "ARP.h"
		#include "Webserver.h"
		#include "Telnet.h"

		#include <MyUSB/Version.h>                        // Library Version Information
		#include <MyUSB/Common/ButtLoadTag.h>             // PROGMEM tags readable by the ButtLoad project
		#include <MyUSB/Drivers/USB/USB.h>                // USB Functionality
		#include <MyUSB/Drivers/Board/Joystick.h>         // Joystick driver
		#include <MyUSB/Drivers/Board/LEDs.h>             // LEDs driver
		#include <MyUSB/Scheduler/Scheduler.h>            // Simple scheduler for task management

		#include <MyUSB/Drivers/AT90USBXXX/Serial_Stream.h>
	
	/* Event Handlers: */
		HANDLES_EVENT(USB_Connect);
		HANDLES_EVENT(USB_Disconnect);
		HANDLES_EVENT(USB_ConfigurationChanged);
		HANDLES_EVENT(USB_UnhandledControlPacket);

	/* Type Defines: */
		typedef struct
		{
			uint8_t  bmRequestType;
			uint8_t  bNotification;
			uint16_t wValue;
			uint16_t wIndex;
			uint16_t wLength;
		} USB_Notification_t;
		
	/* Tasks: */
		TASK(RNDIS_Task);
		TASK(Ethernet_Task);
		
#endif
