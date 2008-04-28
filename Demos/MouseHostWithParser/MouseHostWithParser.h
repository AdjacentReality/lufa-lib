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
		#include <MyUSB/Drivers/USB/Class/HIDParser.h>            // HID Class Report Parser
		#include <MyUSB/Drivers/USB/USB.h>                        // USB Functionality
		#include <MyUSB/Drivers/AT90USBXXX/Serial_Stream.h>       // Serial stream driver
		#include <MyUSB/Drivers/Board/LEDs.h>                     // LEDs driver
		#include <MyUSB/Scheduler/Scheduler.h>                    // Simple scheduler for task management
		
	/* Macros: */
		#define MOUSE_DATAPIPE              1
		#define MOUSE_CLASS                 0x03
		#define MOUSE_PROTOCOL              0x02

		#define DTYPE_HID                   0x21
		#define DTYPE_Report                0x22
		
		#define USAGE_PAGE_BUTTON           0x09
		#define USAGE_PAGE_GENERIC_DCTRL    0x01
		#define USAGE_X                     0x30
		#define USAGE_Y                     0x31

		#define MAX_CONFIG_DESCRIPTOR_SIZE  512
		
	/* Enums: */
		enum GetConfigDescriptorDataCodes_t
		{
			ControlError         = 0,
			DescriptorTooLarge   = 1,
			NoHIDInterfaceFound  = 2,
			NoHIDDescriptorFound = 3,
			NoEndpointFound      = 4,
			SuccessfulConfigRead = 5,
		};

		enum GetHIDReportDataCodes_t
		{
			ParseSucessful          = 0,
			ParseError              = 1,
			ParseControlError       = 2,
		};

	/* Type Defines: */
		typedef struct
		{
			USB_Descriptor_Header_t  Header;
				
			uint16_t                 HIDSpec;
			uint8_t                  CountryCode;
		
			uint8_t                  TotalHIDDescriptors;

			uint8_t                  HIDReportType;
			uint16_t                 HIDReportLength;
		} USB_Descriptor_HID_t;

	/* Task Definitions: */
		TASK(USB_Mouse_Host);

	/* Event Handlers: */
		HANDLES_EVENT(USB_DeviceAttached);
		HANDLES_EVENT(USB_DeviceUnattached);
		HANDLES_EVENT(USB_HostError);
		HANDLES_EVENT(USB_DeviceEnumerationFailed);
		
	/* Function Prototypes: */
		uint8_t GetHIDReportData(void);
		uint8_t GetConfigDescriptorData(void);
		
#endif
