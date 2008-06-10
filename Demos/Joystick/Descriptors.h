/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

	/* Includes: */
		#include <MyUSB/Drivers/USB/USB.h>

		#include <avr/pgmspace.h>

	/* Type Defines: */
		typedef struct
		{
			USB_Descriptor_Header_t               Header;
				
			uint16_t                              HIDSpec;
			uint8_t                               CountryCode;
		
			uint8_t                               TotalHIDReports;

			uint8_t                               HIDReportType;
			uint16_t                              HIDReportLength;
		} USB_Descriptor_HID_t;

		typedef uint8_t USB_Descriptor_HIDReport_Datatype_t;

		typedef struct
		{
			USB_Descriptor_Configuration_Header_t Config;
			USB_Descriptor_Interface_t            Interface;
			USB_Descriptor_HID_t                  JoystickHID;
	        USB_Descriptor_Endpoint_t             JoystickEndpoint;
		} USB_Descriptor_Configuration_t;
					
	/* Macros: */
		#define JOYSTICK_EPNUM               1
		#define JOYSTICK_EPSIZE              8

		#define DTYPE_HID                    0x21
		#define DTYPE_Report                 0x22

	/* External Variables: */
		extern USB_Descriptor_HIDReport_Datatype_t JoystickReport[];
		extern USB_Descriptor_Configuration_t      ConfigurationDescriptor;

	/* Function Prototypes: */
		bool USB_GetDescriptor(const uint16_t wValue, const uint8_t wIndex,
		                       void** const DescriptorAddress, uint16_t* const DescriptorSize)
		                       ATTR_WARN_UNUSED_RESULT ATTR_WEAK ATTR_NON_NULL_PTR_ARG(3, 4);

#endif
