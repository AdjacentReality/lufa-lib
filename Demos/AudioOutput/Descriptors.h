/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the GPL Licence, Version 3
*/

#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

	/* Includes: */
		#include <MyUSB/Drivers/USB/USB.h>

	/* Type Defines: */
		typedef struct
		{
			USB_Descriptor_Configuration_Header_t Config;
		} USB_Descriptor_Configuration_t;
					
	/* Function Prototypes: */
		bool USB_GetDescriptor(const uint8_t Type, const uint8_t Index,
							   void** const DescriptorAddr, uint16_t* const Size)
							   ATTR_WARN_UNUSED_RESULT;

#endif
