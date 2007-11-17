/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the GPL Licence, Version 3
*/

#if !defined(USB_DEVICE_ONLY) // All modes or USB_HOST_ONLY
#include "Host.h"

USB_Host_Request_Header_t USB_HostRequest;

uint8_t USB_Host_SendControlRequest(void)
{
	uint8_t* HeaderByte = (uint8_t*)&USB_HostRequest;

	if (USB_Host_WaitMS(1) != HOST_WAITERROR_Sucessful)
	  return SEND_CONTROL_ERROR;

	Pipe_SelectPipe(0);
	Pipe_SetToken(PIPE_TOKEN_SETUP);
	Pipe_ClearSetupSent();

	Pipe_Unfreeze();

	for (uint8_t i = 0; i < sizeof(USB_Host_Request_Header_t); i++)
	  USB_Host_Write_Byte(*(HeaderByte++));

	Pipe_SendPipeData();
	
	while (!(Pipe_IsSetupSent()))
	{
		if (USB_Host_WaitMS(1) != HOST_WAITERROR_Sucessful)
		{
			Pipe_Freeze();
			return SEND_CONTROL_ERROR;
		}
	}
	

	if ((USB_HostRequest.RequestType & CONTROL_REQTYPE_DIRECTION) == REQDIR_HOSTTODEVICE)
	{
		Pipe_SetToken(PIPE_TOKEN_OUT);
		Pipe_Unfreeze();

		while (!(Pipe_IsSetupOutRecieved()) && !(Pipe_IsSetupStalled()));		
	}
	else
	{
		Pipe_SetInfiniteINRequests();
		Pipe_SetToken(PIPE_TOKEN_IN);
		Pipe_Unfreeze();
		
		while (!(Pipe_IsSetupInRecieved()) && !(Pipe_IsSetupStalled()));		
	}

	if (Pipe_IsSetupStalled())
	{
		Pipe_ClearSetupStalled();
		return SEND_CONTROL_ERROR;
	}

	Pipe_Freeze();
	
	return SEND_CONTROL_OK;
};
#endif
