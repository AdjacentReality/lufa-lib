/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the GPL Licence, Version 3
*/

/*
	Keyboard demonstration application by Denver Gingerich.

	This example is based on the MyUSB Mouse demonstration application,
	written by Dean Camera.

	Keyboard report descriptor is from the AVR-USB project, used with
	permission.
*/

/*
	Keyboard demonstration application. This gives a simple reference
	application for implementing a USB Keyboard using the basic USB HID
	drivers in all modern OSes (i.e. no special drivers required).
	
	On startup the system will automatically enumerate and function
	as a keyboard when the USB connection to a host is present. To use
	the keyboard example, manipulate the joystick to send the letters
	A, B, C, D and E. See the USB HID documentation for more information
	on sending keyboard event and keypresses.
*/

#include "Keyboard.h"

TASK_ID_LIST
{
	USB_USBTask_ID,
	USB_Keyboard_Report_ID,
};

TASK_LIST
{
	{ TaskID: USB_USBTask_ID          , TaskName: USB_USBTask          , TaskStatus: TASK_RUN  },
	{ TaskID: USB_Keyboard_Report_ID  , TaskName: USB_Keyboard_Report  , TaskStatus: TASK_RUN  },
};

USB_HID_Report_Keyboard_t KeyboardReport PROGMEM =
{
	ReportData:
	{
		0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
		0x09, 0x06,                    // USAGE (Keyboard)
		0xa1, 0x01,                    // COLLECTION (Application)
		0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
		0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
		0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
		0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
		0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
		0x75, 0x01,                    //   REPORT_SIZE (1)
		0x95, 0x08,                    //   REPORT_COUNT (8)
		0x81, 0x02,                    //   INPUT (Data,Var,Abs)
		0x95, 0x01,                    //   REPORT_COUNT (1)
		0x75, 0x08,                    //   REPORT_SIZE (8)
		0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
		0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
		0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
		0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
		0xc0                           // END_COLLECTION
	}
};

int main(void)
{
	/* Disable Clock Division */
	CLKPR = (1 << CLKPCE);
	CLKPR = 0;

	/* Hardware Initialization */
	Joystick_Init();
	Bicolour_Init();
	
	/* Initial LED colour - Double red to indicate USB not ready */
	Bicolour_SetLeds(BICOLOUR_LED1_RED | BICOLOUR_LED2_RED);
	
	/* Initialize USB Subsystem */
	USB_Init(USB_MODE_DEVICE, USB_DEV_HIGHSPEED);

	/* Scheduling */
	Scheduler_Start(); // Scheduler never returns, so put this last
}

EVENT_HANDLER(USB_CreateEndpoints)
{
	Endpoint_ConfigureEndpoint(KEYBOARD_EPNUM, ENDPOINT_TYPE_INTERRUPT,
		                       ENDPOINT_DIR_IN, KEYBOARD_EPSIZE,
	                           ENDPOINT_BANK_SINGLE);

	Bicolour_SetLeds(BICOLOUR_LED1_GREEN | BICOLOUR_LED2_GREEN);
}

TASK(USB_Keyboard_Report)
{
	USB_KeyboardReport_Data_t KeyboardReportData = {Modifier: 0, KeyCode: 0};
	uint8_t                   JoyStatus_LCL      = Joystick_GetStatus();


	if (JoyStatus_LCL & JOY_UP)
		KeyboardReportData.KeyCode = 0x04; // A
	else if (JoyStatus_LCL & JOY_DOWN)
		KeyboardReportData.KeyCode = 0x05; // B

	if (JoyStatus_LCL & JOY_LEFT)
		KeyboardReportData.KeyCode = 0x06; // C
	else if (JoyStatus_LCL & JOY_RIGHT)
		KeyboardReportData.KeyCode = 0x07; // D

	if (JoyStatus_LCL & JOY_PRESS)
		KeyboardReportData.KeyCode = 0x08; // E

	if (USB_IsConnected && USB_IsInitialized)
	{
		Endpoint_SelectEndpoint(KEYBOARD_EPNUM);

		if (Endpoint_ReadWriteAllowed())
		{
			USB_Device_Write_Byte(KeyboardReportData.Modifier);
			USB_Device_Write_Byte(KeyboardReportData.KeyCode);
			
			USB_In_Clear();
		}
	}
}

