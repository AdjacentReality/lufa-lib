/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the GPL Licence, Version 3
*/

/*
	Keyboard host demonstration application, using pipe interrupts. This gives
	a simple reference application for implementing a USB Keyboard host utilizing
	the MyUSB pipe interrupt system, for USB keyboards using the standard Keyboard
	HID profile.
	
	Pressed alpha-numeric, enter or space key is transmitted through the serial
	USART at serial settings 9600, 8, N, 1.

	Currently only single-interface keyboards are supported.
*/

#include "KeyboardHostViaInt.h"

/* Project Tags, for reading out using the ButtLoad project */
BUTTLOADTAG(ProjName,  "MyUSB Keyboard Host App");
BUTTLOADTAG(BuildTime, __TIME__);
BUTTLOADTAG(BuildDate, __DATE__);

/* Scheduler Task ID list */
TASK_ID_LIST
{
	USB_USBTask_ID,
	USB_Keyboard_Host_ID,
};

/* Scheduler Task List */
TASK_LIST
{
	{ TaskID: USB_USBTask_ID          , TaskName: USB_USBTask          , TaskStatus: TASK_RUN  },
	{ TaskID: USB_Keyboard_Host_ID    , TaskName: USB_Keyboard_Host    , TaskStatus: TASK_RUN  },
};

int main(void)
{
	/* Disable Clock Division */
	CLKPR = (1 << CLKPCE);
	CLKPR = 0;

	/* Hardware Initialization */
	SerialStream_Init(9600);
	Bicolour_Init();
	
	/* Initial LED colour - Double red to indicate USB not ready */
	Bicolour_SetLeds(BICOLOUR_LED1_RED | BICOLOUR_LED2_RED);
	
	/* Initialize USB Subsystem */
	USB_Init(USB_MODE_HOST, USB_OPT_REG_ENABLED);

	/* Startup message */
	puts_P(PSTR(ESC_RESET ESC_BG_WHITE ESC_INVERSE_ON ESC_ERASE_DISPLAY
	       "Keyboard Host Demo running.\r\n" ESC_INVERSE_OFF));
		   
	/* Scheduling - routine never returns, so put this last in the main function */
	Scheduler_Start();
}

EVENT_HANDLER(USB_DeviceAttached)
{
	puts_P(PSTR("Device Attached.\r\n"));
	Bicolour_SetLeds(BICOLOUR_NO_LEDS);	
}

EVENT_HANDLER(USB_DeviceUnattached)
{
	puts_P(PSTR("\r\nDevice Unattached.\r\n"));
	Bicolour_SetLeds(BICOLOUR_LED1_RED | BICOLOUR_LED2_RED);
}

EVENT_HANDLER(USB_HostError)
{
	USB_ShutDown();

	puts_P(PSTR(ESC_BG_RED "Host Mode Error\r\n"));
	printf_P(PSTR(" -- Error Code %d\r\n"), ErrorCode);

	Bicolour_SetLeds(BICOLOUR_LED1_RED | BICOLOUR_LED2_RED);
	for(;;);
}

TASK(USB_Keyboard_Host)
{
	static uint8_t DataBuffer[sizeof(USB_Descriptor_Configuration_Header_t) +
				              sizeof(USB_Descriptor_Interface_t)];

	/* Block task if device not connected */
	if (!(USB_IsConnected))
		return;

	switch (USB_HostState)
	{
		case HOST_STATE_Addressed:
			/* Standard request to set the device configuration to configuration 1 */
			USB_HostRequest.RequestType    = (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE);
			USB_HostRequest.RequestData    = REQ_SetConfiguration;
			USB_HostRequest.Value          = 1;
			USB_HostRequest.Index          = 0;
			USB_HostRequest.Length         = USB_ControlPipeSize;

			/* Send the request, display error and wait for device detatch if request fails */
			if (USB_Host_SendControlRequest(NULL) != HOST_SENDCONTROL_Sucessful)
			{
				puts_P(PSTR("Control error."));

				Bicolour_SetLeds(BICOLOUR_LED1_RED);
				while (USB_IsConnected);
				break;
			}
		
			USB_HostState = HOST_STATE_Configured;
			break;
		case HOST_STATE_Configured:
			/* Standard request to retrieve Configuration descriptor from device */
			USB_HostRequest.RequestType    = (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE);
			USB_HostRequest.RequestData    = REQ_GetDescriptor;
			USB_HostRequest.Value_HighByte = DTYPE_Configuration;
			USB_HostRequest.Value_LowByte  = 0;
			USB_HostRequest.Index          = 0;
			USB_HostRequest.Length         = sizeof(DataBuffer);

			/* Send the request, display error and wait for device detatch if request fails */
			if (USB_Host_SendControlRequest(DataBuffer)
			    != HOST_SENDCONTROL_Sucessful)
			{
				puts_P(PSTR("Control error."));
			
				Bicolour_SetLeds(BICOLOUR_LED1_RED);
				while (USB_IsConnected);
				break;
			}

			/* Check Device descriptor's interface class against the keyboard class */
			if (DataBuffer[sizeof(USB_Descriptor_Configuration_Header_t) +
			               offsetof(USB_Descriptor_Interface_t, Class)] != KEYBOARD_CLASS)
			{
				puts_P(PSTR("Incorrect device class."));

				Bicolour_SetLeds(BICOLOUR_LED1_RED);
				while (USB_IsConnected);
				break;
			}
				
			/* Check Device descriptor's interface protocol against the keyboard protocol */
			if (DataBuffer[sizeof(USB_Descriptor_Configuration_Header_t) +
			               offsetof(USB_Descriptor_Interface_t, Protocol)] != KEYBOARD_PROTOCOL)
			{
				puts_P(PSTR("Incorrect device protocol."));

				Bicolour_SetLeds(BICOLOUR_LED1_RED);
				while (USB_IsConnected);
				break;
			}

			/* Configure the keyboard data pipe */
			Pipe_ConfigurePipe(KEYBOARD_DATAPIPE, PIPE_TYPE_INTERRUPT, PIPE_TOKEN_IN, 1, 8, PIPE_BANK_SINGLE);
			Pipe_SelectPipe(KEYBOARD_DATAPIPE);
			Pipe_SetInfiniteINRequests();
			
			/* Activate the keyboard data pipe IN interrupt */
			Pipe_SetInterruptFreq(1);
			Pipe_Unfreeze();
			USB_INT_Enable(PIPE_INT_IN);
			
			puts_P(PSTR("Keyboard Enumerated.\r\n"));

			USB_HostState = HOST_STATE_Ready;
			break;
	}
}

ISR(ENDPOINT_PIPE_vect)
{
	USB_KeyboardReport_Data_t KeyboardReport;
	char                      PressedKey = 0;

	/* Check to see if the keyboard data pipe has caused the interrupt */
	if (Pipe_HasPipeInterrupted(KEYBOARD_DATAPIPE))
	{
		/* Clear the pipe interrupt, and select the keyboard pipe */
		Pipe_ClearPipeInterrupt(KEYBOARD_DATAPIPE);
		Pipe_SelectPipe(KEYBOARD_DATAPIPE);	

		/* Check to see if the pipe IN interrupt has fired */
		if (USB_INT_HasOccurred(PIPE_INT_IN) && USB_INT_IsEnabled(PIPE_INT_IN))
		{
			/* Read in keyboard report data */
			KeyboardReport.Modifier = Pipe_Read_Byte();
			Pipe_Ignore_Byte();
			KeyboardReport.KeyCode  = Pipe_Read_Byte();
						
			Bicolour_SetLed(BICOLOUR_LED1, (KeyboardReport.Modifier) ? BICOLOUR_LED1_RED
			                                                         : BICOLOUR_LED1_OFF);
						
			/* Check if a key has been pressed */
			if (KeyboardReport.KeyCode)
			{
				/* Toggle status LED to indicate keypress */
				if (Bicolour_GetLeds() & BICOLOUR_LED2_GREEN)
				  Bicolour_TurnOffLeds(BICOLOUR_LED2_GREEN);
				else
				  Bicolour_TurnOnLeds(BICOLOUR_LED2_GREEN);

				/* Retrieve pressed key character if alphanumeric */
				if ((KeyboardReport.KeyCode >= 0x04) && (KeyboardReport.KeyCode <= 0x1D))
				  PressedKey = (KeyboardReport.KeyCode - 0x04) + 'A';
				else if ((KeyboardReport.KeyCode >= 0x1E) && (KeyboardReport.KeyCode <= 0x27))
				  PressedKey = (KeyboardReport.KeyCode - 0x1E) + '0';
				else if (KeyboardReport.KeyCode == 0x2C)
				  PressedKey = ' ';						
				else if (KeyboardReport.KeyCode == 0x28)
				  PressedKey = '\n';
						 
				/* Print the pressed key character out through the serial port if valid */
				if (PressedKey)
				  printf_P(PSTR("%c"), PressedKey);
			}

			/* Clear the IN endpoint and interrupt flag, ready for next data packet */
			Pipe_In_Clear();
			USB_INT_Clear(PIPE_INT_IN);		
		}
	}
}
