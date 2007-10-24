/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the GPL Licence, Version 3
*/

#ifndef DEVCHAP9_H
#define DEVCHAP9_H

	/* Includes: */
		#include <avr/io.h>
		#include <avr/pgmspace.h>
		
		#include "../../../Configuration/USB/Device/Descriptors.h"
		#include "../../../Configuration/USB/Device/UserDeviceRoutines.h"
		#include "LowLevel.h"
		#include "StdRequestType.h"

	/* External Variables */
		extern uint8_t USB_ConfigurationNumber;

	/* Function Prototypes */
		void USB_DEVC9_ProcessControlPacket(void);
		void USB_DEVC9_SetAddress(void);
		void USB_DEVC9_SetConfiguration(void);
		void USB_DEVC9_GetConfiguration(void);
		void USB_DEVC9_GetDescriptor(void);
		void USB_DEVC9_GetStatus(const uint8_t RequestType);
		void USB_DEVC9_SetFeature(const uint8_t RequestType);
		void USB_DEVC9_ClearFeature(const uint8_t RequestType);
		
#endif
