/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

#if ((defined(__AVR_AT90USB1286__) || (defined(__AVR_AT90USB646__))) && !(defined(USB_DEVICE_ONLY)))
	#define USB_DEVICE_ONLY
#endif

#if !defined(USB_DEVICE_ONLY) // All modes or USB_HOST_ONLY
#include "ConfigDescriptor.h"

uint8_t AVR_HOST_GetDeviceConfigDescriptorSize(uint16_t* const ConfigSizePtr)
{
	uint8_t ErrorCode;
	uint8_t Buffer[sizeof(USB_Descriptor_Configuration_Header_t)];

	USB_HostRequest = (USB_Host_Request_Header_t)
		{
			RequestType: (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE),
			RequestData: REQ_GetDescriptor,
			Value:       (DTYPE_Configuration << 8),
			Index:       0,
			DataLength:  sizeof(USB_Descriptor_Configuration_Header_t),
		};
		
	ErrorCode = USB_Host_SendControlRequest(Buffer);
	
	*ConfigSizePtr = ((USB_Descriptor_Configuration_Header_t*)&Buffer)->TotalConfigurationSize;

	return ErrorCode;
}

uint8_t AVR_HOST_GetDeviceConfigDescriptor(const uint16_t BufferSize, uint8_t* const BufferPtr)
{
	USB_HostRequest = (USB_Host_Request_Header_t)
		{
			RequestType: (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE),
			RequestData: REQ_GetDescriptor,
			Value:       (DTYPE_Configuration << 8),
			Index:       0,
			DataLength:  BufferSize,
		};

	return USB_Host_SendControlRequest(BufferPtr);
}
#endif

