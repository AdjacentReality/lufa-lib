/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the GPL Licence, Version 3
*/

#include "USBInterrupt.h"

ISR(USB_GEN_vect)
{
	if (USB_INT_OCCURRED(USB_INT_VBUS) && USB_INT_ISENABLED(USB_INT_VBUS))
	{
		RAISE_EVENT(USB_VBUSChange);

		if (USB_VBUS_GetStatus())
		  RAISE_EVENT(USB_VBUSConnect);

		if (!(USB_IsConnected) && USB_VBUS_GetStatus() && USB_IsInitialized)
		{
			if (USB_SetupInterface() == USB_SETUPINTERFACE_OK)
			{
				USB_IsConnected = true;
				
				RAISE_EVENT(USB_Connect);
			}
		}
		else
		{
			if (USB_CurrentMode == USB_MODE_DEVICE)
				USB_Detach();
			
			RAISE_EVENT(USB_VBUSDisconnect);
		
			USB_IsConnected = false;
		}
		
		USB_INT_CLEAR(USB_INT_VBUS);
	}

	if (USB_INT_OCCURRED(USB_INT_SUSPEND) && USB_INT_ISENABLED(USB_INT_SUSPEND))
	{
		USB_CLK_Freeze();
		USB_PLL_Off();

		USB_INT_CLEAR(USB_INT_SUSPEND);
		USB_INT_DISABLE(USB_INT_SUSPEND);
		USB_INT_ENABLE(USB_INT_WAKEUP);
		
		RAISE_EVENT(USB_Suspend);
	}

	if (USB_INT_OCCURRED(USB_INT_WAKEUP) && USB_INT_ISENABLED(USB_INT_WAKEUP))
	{
		USB_CLK_Unfreeze();
		USB_PLL_On();
		while (!(USB_PLL_IsReady()));

		USB_INT_CLEAR(USB_INT_WAKEUP);
		USB_INT_DISABLE(USB_INT_WAKEUP);
		USB_INT_ENABLE(USB_INT_SUSPEND);
		
		RAISE_EVENT(USB_WakeUp);
	}
   
	if (USB_INT_OCCURRED(USB_INT_EORSTI) && USB_INT_ISENABLED(USB_INT_EORSTI))
	{
		USB_INT_CLEAR(USB_INT_EORSTI);

		USB_ConfigurationNumber = 0;
		USB_INT_CLEAR(USB_INT_SUSPEND);
		USB_INT_DISABLE(USB_INT_SUSPEND);
		USB_INT_ENABLE(USB_INT_WAKEUP);

		Endpoint_ConfigureEndpoint(ENDPOINT_CONTROLEP, ENDPOINT_TYPE_CONTROL,
		                           ENDPOINT_DIR_OUT, ENDPOINT_CONTROLEP_SIZE,
		                           ENDPOINT_BANK_SINGLE);

		RAISE_EVENT(USB_Reset);
	}
	
	if (USB_INT_OCCURRED(USB_INT_IDTI) && USB_INT_ISENABLED(USB_INT_IDTI))
	{		
		RAISE_EVENT(USB_UIDChange);
		RAISE_EVENT(USB_Disconnect);

		USB_INT_CLEAR(USB_INT_IDTI);
		
		USB_SetupInterface();
	}
	
	if (USB_INT_OCCURRED(USB_INT_DDISCI) && USB_INT_ISENABLED(USB_INT_DDISCI))
	{
		USB_IsConnected = false;
		RAISE_EVENT(USB_Disconnect);

		USB_INT_CLEAR(USB_INT_DDISCI);

		USB_HostState = HOST_STATE_Unattached;
	}
}
