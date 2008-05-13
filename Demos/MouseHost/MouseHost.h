/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

#ifndef _MOUSE_HOST_H_
#define _MOUSE_HOST_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/pgmspace.h>
		#include <stdio.h>

		#include <MyUSB/Version.h>                                // Library Version Information
		#include <MyUSB/Common/ButtLoadTag.h>                     // PROGMEM tags readable by the ButtLoad project
		#include <MyUSB/Drivers/Misc/TerminalCodes.h>             // ANSI Terminal Escape Codes
		#include <MyUSB/Drivers/USB/USB.h>                        // USB Functionality
		#include <MyUSB/Drivers/USB/Class/ConfigDescriptor.h>     // Configuration Descriptor Parser
		#include <MyUSB/Drivers/AT90USBXXX/Serial_Stream.h>       // Serial stream driver
		#include <MyUSB/Drivers/Board/LEDs.h>                     // LEDs driver
		#include <MyUSB/Scheduler/Scheduler.h>                    // Simple scheduler for task management
		
	/* Macros: */
		#define MOUSE_DATAPIPE              1
		#define MOUSE_CLASS                 0x03
		#define MOUSE_PROTOCOL              0x02

		#define MAX_CONFIG_DESCRIPTOR_SIZE  512

	/* Type Defines: */
		typedef struct
		{
			uint8_t Button;
			int8_t  X;
			int8_t  Y;
		} USB_MouseReport_Data_t;
		
	/* Enums: */
		enum GetConfigDescriptorDataCodes_t
		{
			ControlError         = 0,
			DescriptorTooLarge   = 1,
			NoHIDInterfaceFound  = 2,
			NoEndpointFound      = 3,
			SuccessfulConfigRead = 4,
		};

	/* Task Definitions: */
		TASK(USB_Mouse_Host);

	/* Configuration Descriptor Comparison Functions: */
		DESCRIPTOR_COMPARATOR(NextMouseInterface);
		DESCRIPTOR_COMPARATOR(NextInterfaceMouseDataEndpoint);

	/* Event Handlers: */
		HANDLES_EVENT(USB_DeviceAttached);
		HANDLES_EVENT(USB_DeviceUnattached);
		HANDLES_EVENT(USB_HostError);
		HANDLES_EVENT(USB_DeviceEnumerationFailed);
		
	/* Function Prototypes: */
		uint8_t ProcessConfigurationDescriptor(void);
		
#endif
