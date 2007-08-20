/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the GPL Licence, Version 3
*/

#include "LowLevel.h"

volatile uint8_t USB_CurrentMode = USB_MODE_NONE;
         uint8_t USB_Options;

void USB_Init(const uint8_t Mode, const uint8_t Options)
{
	if (USB_IsInitialized)
	  USB_ShutDown();

	if (Mode == USB_MODE_UID)
	{
		USB_INT_ENABLE(USB_INT_IDTI);
		USB_INT_CLEAR(USB_INT_IDTI);
	}
	else if (Mode == USB_MODE_DEVICE)
	{
		UHWCON |=  (1 << UIMOD);
		
		USB_CurrentMode = USB_MODE_DEVICE;
	}
	else if (Mode == USB_MODE_HOST)			
	{
		UHWCON &= ~(1 << UIMOD);
		
		USB_CurrentMode = USB_MODE_HOST;
	}

	if (UHWCON & (1 << UIDE))
	  USB_CurrentMode = USB_GetUSBModeFromUID();

	USB_InitTaskPointer();
	USB_OTGPAD_On();
	
	USB_Options = Options;
		
	if (USB_CurrentMode == USB_MODE_DEVICE)
	  USB_INT_ENABLE(USB_INT_VBUS);
	else
	  USB_SetupInterface();
	
	sei();
}

void USB_ShutDown(void)
{
	if (USB_IsConnected)
	  USB_EVENT_OnUSBDisconnect();

	USB_INT_DisableAllInterrupts();
	Endpoint_ClearEndpoints();
	Pipe_ClearPipes();

	USB_CLK_Freeze();

	USB_Interface_Disable();
	USB_PLL_Off();
	USB_REG_Off();
	USB_OTGPAD_Off();

	UHWCON &= ~(1 << UIDE);

	USB_IsConnected   = false;
	USB_IsInitialized = false;
	USB_ConfigurationNumber = 0;
}

bool USB_SetupInterface(void)
{
	USB_IsConnected   = false;
	USB_IsInitialized = false;
	USB_ConfigurationNumber = 0;
	
	if (USB_IsConnected)
	  USB_EVENT_OnUSBDisconnect();
	
	USB_HOST_HostModeOff();
	USB_Detach();

	USB_INT_DisableAllInterrupts();

	Endpoint_ClearEndpoints();
	Pipe_ClearPipes();

	if (UHWCON & (1 << UIDE))
	  USB_CurrentMode = USB_GetUSBModeFromUID();
	  
	if (USB_CurrentMode == USB_MODE_NONE)
	{
		USB_EVENT_PowerOnFail("N/A", POWERON_ERR_NoModeSpecified);
		return USB_SETUPINTERFACE_FAIL;
	}
	else if (USB_CurrentMode == USB_MODE_DEVICE)
	{
		if (USB_Options & USB_DEV_LOWSPEED)
		  USB_DEV_SetLowSpeed();
		else
		  USB_DEV_SetHighSpeed();
		  
		USB_INT_ENABLE(USB_INT_VBUS);
	}
	
	USB_REG_On();
	USB_PLL_On();
			
	while (!(USB_PLL_IsReady()));
		
	USB_Interface_Disable();
	USB_Interface_Enable();
	USB_CLK_Unfreeze();

	if (USB_CurrentMode == USB_MODE_DEVICE)
	{
		if (Endpoint_ConfigureEndpoint(ENDPOINT_CONTROLEP, ENDPOINT_TYPE_CONTROL,
		                               ENDPOINT_DIR_OUT, ENDPOINT_CONTROLEP_SIZE,
									   ENDPOINT_BANK_SINGLE) == ENDPOINT_CONFIG_OK)
		{
			USB_Attach();
		}
		else
		{
			USB_EVENT_PowerOnFail("DEVICE", POWERON_ERR_EndpointCreationFailed);
			return USB_SETUPINTERFACE_FAIL;
		}

		USB_INT_ENABLE(USB_INT_SUSPEND);
		USB_INT_ENABLE(USB_INT_EORSTI);
	}
	else if (USB_CurrentMode == USB_MODE_HOST)
	{
		if (Pipe_ConfigurePipe(PIPE_CONTROLPIPE, PIPE_TYPE_CONTROL,
		                       PIPE_TOKEN_SETUP, PIPE_CONTROLPIPE,
							   PIPE_CONTROLPIPE_SIZE, PIPE_BANK_SINGLE)
							   == PIPE_CONFIG_OK)
		{
			USB_Attach();
			USB_HOST_HostModeOn();
		}
		else
		{
			USB_EVENT_PowerOnFail("MASTER", POWERON_ERR_PipeCreationFailed);
			return USB_SETUPINTERFACE_FAIL;
		}
	}
	
	USB_InitTaskPointer();

	return USB_SETUPINTERFACE_OK;
}
