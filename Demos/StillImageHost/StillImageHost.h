/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

#ifndef _STILL_IMAGE_HOST_H_
#define _STILL_IMAGE_HOST_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <stdio.h>
		
		#include "ConfigDescriptor.h"
		#include "PIMACodes.h"
		#include "StillImageCommands.h"

		#include <MyUSB/Common/ButtLoadTag.h>                     // PROGMEM tags readable by the ButtLoad project
		#include <MyUSB/Drivers/Misc/TerminalCodes.h>             // ANSI Terminal Escape Codes
		#include <MyUSB/Drivers/USB/USB.h>                        // USB Functionality
		#include <MyUSB/Drivers/AT90USBXXX/Serial_Stream.h>       // Serial stream driver
		#include <MyUSB/Drivers/Board/LEDs.h>                     // LED driver
		#include <MyUSB/Scheduler/Scheduler.h>                    // Simple scheduler for task management
		
	/* Macros: */
		#define SIMAGE_CLASS                   0x06
		#define SIMAGE_SUBCLASS                0x01
		#define SIMAGE_PROTOCOL                0x01

	/* Task Definitions: */
		TASK(USB_SImage_Host);

	/* Event Handlers: */
		HANDLES_EVENT(USB_DeviceAttached);
		HANDLES_EVENT(USB_DeviceUnattached);
		HANDLES_EVENT(USB_DeviceEnumerationComplete);
		HANDLES_EVENT(USB_HostError);
		HANDLES_EVENT(USB_DeviceEnumerationFailed);
		
	/* Function Prototypes: */
		void UnicodeToASCII(uint8_t* UnicodeString, char* Buffer);
		void ShowCommandError(uint8_t ErrorCode, bool ResponseCodeError);
		
#endif
