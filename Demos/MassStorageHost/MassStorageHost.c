/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the GPL Licence, Version 3
*/

/*
	Mass Storage host demonstration application. This gives a simple reference
	application for implementing a USB Mass Storage host, for USB storage devices
	using the standard Mass Storage USB profile.
	
	The first 512 bytes of an attached disk's memory will be dumped out of the
	serial port when it is attached to the AT90USB1287 AVR.
	
	Requires header files from the Mass Storage Device demonstation application.
*/

/*
	THIS EXAMPLE IS UNFINISHED AND NON OPERATIONAL. FOR DEVELOPMENT PURPOSES ONLY.
*/

#include "MassStorageHost.h"

/* Project Tags, for reading out using the ButtLoad project */
BUTTLOADTAG(ProjName,  "MyUSB MS Host App");
BUTTLOADTAG(BuildTime, __TIME__);
BUTTLOADTAG(BuildDate, __DATE__);

/* Scheduler Task ID list */
TASK_ID_LIST
{
	USB_USBTask_ID,
	USB_MassStore_Host_ID,
};

/* Scheduler Task List */
TASK_LIST
{
	{ TaskID: USB_USBTask_ID          , TaskName: USB_USBTask          , TaskStatus: TASK_RUN  },
	{ TaskID: USB_MassStore_Host_ID   , TaskName: USB_MassStore_Host   , TaskStatus: TASK_RUN  },
};

/* Globals */
uint8_t  MassStoreEndpointNumber_IN;
uint8_t  MassStoreEndpointNumber_OUT;
uint16_t MassStoreEndpointSize_IN;
uint16_t MassStoreEndpointSize_OUT;

volatile unsigned long TEST_GLOBAL;

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
	       "MassStore Host Demo running.\r\n" ESC_INVERSE_OFF));
		   
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

TASK(USB_MassStore_Host)
{
	uint8_t ErrorCode;

	/* Block task if device not connected */
	if (!(USB_IsConnected))
		return;

	switch (USB_HostState)
	{
		case HOST_STATE_Addressed:
			/* Standard request to set the device configuration to configuration 1 */
			USB_HostRequest = (USB_Host_Request_Header_t)
				{
					RequestType: (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE),
					RequestData: REQ_SetConfiguration,
					Value:       1,
					Index:       0,
					Length:      USB_ControlPipeSize,
				};

			/* Send the request, display error and wait for device detatch if request fails */
			if (USB_Host_SendControlRequest(NULL) != HOST_SENDCONTROL_Sucessful)
			{
				puts_P(PSTR("Control error."));

				/* Indicate error via status LEDs */
				Bicolour_SetLeds(BICOLOUR_LED1_RED);

				/* Wait until USB device disconnected */
				while (USB_IsConnected);
				break;
			}
				
			USB_HostState = HOST_STATE_Configured;
			break;
		case HOST_STATE_Configured:
			puts_P(PSTR("Getting Config Data.\r\n"));
		
			/* Get and process the configuration descriptor data */
			ErrorCode = GetConfigDescriptorData();
			
			/* Check if the configuration descriptor processing was sucessful */
			if (ErrorCode != SuccessfulConfigRead)
			{
				switch (ErrorCode)
				{
					case HIDInterfaceNotFound:
						puts_P(PSTR("Invalid Device Type.\r\n"));
						break;
					case IncorrectProtocol:
						puts_P(PSTR("Invalid Protocol.\r\n"));
						break;
					default:
						puts_P(PSTR("Control Error.\r\n"));
						break;
				}

				/* Indicate error via status LEDs */
				Bicolour_SetLeds(BICOLOUR_LED1_RED);
				
				/* Wait until USB device disconnected */
				while (USB_IsConnected);
				break;
			}

			/* Configure the data pipes */
			Pipe_ConfigurePipe(MASS_STORE_DATA_IN_PIPE, EP_TYPE_BULK, PIPE_TOKEN_IN,
			                   1, 64,
			                   PIPE_BANK_DOUBLE);

			Pipe_ConfigurePipe(MASS_STORE_DATA_OUT_PIPE, EP_TYPE_BULK, PIPE_TOKEN_OUT,
			                   2, 64,
			                   PIPE_BANK_DOUBLE);

			Pipe_SelectPipe(MASS_STORE_DATA_IN_PIPE);
			Pipe_SetInfiniteINRequests();
			Pipe_Unfreeze();

			Pipe_SelectPipe(MASS_STORE_DATA_OUT_PIPE);
			Pipe_Unfreeze();
		
			puts_P(PSTR("Mass Storage Disk Enumerated.\r\n"));
				
			USB_HostState = HOST_STATE_Ready;
			break;
		case HOST_STATE_Ready:
			/* Indicate device busy via the status LEDs */
			Bicolour_SetLeds(BICOLOUR_LED2_ORANGE);
			
			if (!(MassStore_PrepareDisk()))
			{
				/* Indicate device error via the status LEDs */
				Bicolour_SetLeds(BICOLOUR_LED1_GREEN | BICOLOUR_LED2_RED);
				
				/* Wait until USB device disconnected */
				while (USB_IsConnected);
				break;
			}

			#if 0
			/* Create a new buffer capabable of holding a single block from the device */
			uint8_t BlockBuffer[DEVICE_BLOCK_SIZE];
						
			/* Read in the first 512 byte block from the device */
			if (!(MassStore_ReadDeviceBlock(0, (uint8_t*)&BlockBuffer)))
			{
				/* Indicate device error via the status LEDs */
				Bicolour_SetLeds(BICOLOUR_LED1_RED | BICOLOUR_LED2_RED);
				
				/* Wait until USB device disconnected */
				while (USB_IsConnected);
				break;
			}
			else
			{
				/* Print the block bytes out through the serial USART */
				for (uint16_t Byte = 0; Byte < DEVICE_BLOCK_SIZE; Byte++)
				  Serial_TxByte(BlockBuffer[Byte]);
			}
			#endif
			
			/* Indicate device no longer busy */
			Bicolour_SetLeds(BICOLOUR_LED2_GREEN);			
			
			/* Wait until USB device disconnected */
			while (USB_IsConnected);
			
			break;
	}
}

uint8_t GetConfigDescriptorData(void)
{
	uint8_t* ConfigDescriptorData;
	uint16_t ConfigDescriptorSize;
	bool     FoundHIDInterfaceDescriptor = false;
	
	/* Get Configuration Descriptor size from the device */
	if (AVR_HOST_GetDeviceConfigDescriptorSize(&ConfigDescriptorSize) != HOST_SENDCONTROL_Sucessful)
	  return ControlError;
	
	/* Ensure that the Configuration Descriptor isn't too large */
	if (ConfigDescriptorSize > MAX_CONFIG_DESCRIPTOR_SIZE)
	  return DescriptorTooLarge;
	  
	/* Allocate enough memory for the entire config descriptor */
	ConfigDescriptorData = __builtin_alloca(ConfigDescriptorSize);

	/* Retrieve the entire configuration descriptor into the allocated buffer */
	AVR_HOST_GetDeviceConfigDescriptor(ConfigDescriptorSize, ConfigDescriptorData);
	
	/* Validate returned data - ensure first entry is a configuration header descriptor */
	if (((USB_Descriptor_Header_t*)ConfigDescriptorData)->Type != DTYPE_Configuration)
	  return ControlError;
	
	while (!(FoundHIDInterfaceDescriptor))
	{
		/* Find next interface descriptor */
		while (ConfigDescriptorSize)
		{
			/* Get the next descriptor from the configuration descriptor data */
			AVR_HOST_GetNextDescriptor(&ConfigDescriptorSize, &ConfigDescriptorData);	  

			/* Check to see if the next descriptor is an interface descriptor, if so break out */
			if (((USB_Descriptor_Header_t*)ConfigDescriptorData)->Type == DTYPE_Interface)
			  break;
		}

		/* If reached end of configuration descriptor, error out */
		if (ConfigDescriptorSize == 0)
		  return HIDInterfaceNotFound;

		/* Check the HID descriptor class, set the flag if class matches expected class */
		if (((USB_Descriptor_Interface_t*)ConfigDescriptorData)->Class == MASS_STORE_CLASS)
		  FoundHIDInterfaceDescriptor = true;
	}

	/* Check subclass - error out if it is incorrect */
	if (((USB_Descriptor_Interface_t*)ConfigDescriptorData)->SubClass != MASS_STORE_SUBCLASS)
	  return IncorrectSubclass;

	/* Check protocol - error out if it is incorrect */
	if (((USB_Descriptor_Interface_t*)ConfigDescriptorData)->Protocol != MASS_STORE_PROTOCOL)
	  return IncorrectProtocol;
	
	/* Find the IN and OUT endpoint descriptors after the mass storage interface descriptor */
	while (ConfigDescriptorSize)
	{
		/* Get the next descriptor from the configuration descriptor data */
		AVR_HOST_GetNextDescriptor(&ConfigDescriptorSize, &ConfigDescriptorData);	  		

		/* Check if current descritor is a BULK type endpoint descriptor */
		if ((((USB_Descriptor_Header_t*)ConfigDescriptorData)->Type == DTYPE_Endpoint) &&
		    (((USB_Descriptor_Endpoint_t*)ConfigDescriptorData)->Attributes == EP_TYPE_BULK))
		{
			uint8_t  EPAddress = ((USB_Descriptor_Endpoint_t*)ConfigDescriptorData)->EndpointAddress;
			uint16_t EPSize    = ((USB_Descriptor_Endpoint_t*)ConfigDescriptorData)->EndpointSize;
		
			/* Set the appropriate endpoint data address based on the endpoint direction */
			if (EPAddress & ENDPOINT_DESCRIPTOR_DIR_IN)
			{
				MassStoreEndpointNumber_IN = EPAddress;
				MassStoreEndpointSize_IN   = EPSize;
			}
			else
			{
				MassStoreEndpointNumber_OUT = EPAddress;
				MassStoreEndpointSize_OUT   = EPSize;
			}
		}
		
		/* If both data pipes found, exit the loop */
		if (MassStoreEndpointNumber_IN && MassStoreEndpointNumber_OUT)
		  break;
		
		/* If new interface descriptor found (indicating the end of the current interface), error out */
		if (((USB_Descriptor_Header_t*)ConfigDescriptorData)->Type == DTYPE_Interface)
		  return NoEndpointFound;
	}	

	/* If reached end of configuration descriptor, error out */
	if (ConfigDescriptorSize == 0)
	  return NoEndpointFound;
	
	/* Valid data found, return success */
	return SuccessfulConfigRead;
}

bool MassStore_PrepareDisk(void)
{
	CommandBlockWrapper_t SCSICommand =
		{
			Header:
				{
					Signature:          CBW_SIGNATURE,
					Tag:                0x01,
					DataTransferLength: 0,
					Flags:              COMMAND_DIRECTION_DATA_OUT,
					LUN:                0x00,
					SCSICommandLength:  6
				},
						
			SCSICommandData:
				{
					SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL,
					0x00,
					0x00,
					0x00,
					0b11,
					0x00
				}
		};
			
	uint8_t* CommandByte = (uint8_t*)&SCSICommand;
			
	/* Select the OUT data pipe for CBW transmission */
	Pipe_SelectPipe(MASS_STORE_DATA_OUT_PIPE);
		
	/* Wait until pipe is ready to be written to */
	while (!(Pipe_Out_IsReady()));

	/* Write the CBW to the OUT pipe */
	for (uint8_t Byte = 0; Byte < sizeof(CommandBlockWrapper_t); Byte++)
	  Pipe_Write_Byte(*(CommandByte++));
	  
	/* Send the data in the OUT pipe to the attached device */
	Pipe_Out_Clear();
	
	/* Select the IN data pipe for data reception */
	Pipe_SelectPipe(MASS_STORE_DATA_IN_PIPE);
		
	/* Wait until data recieved in the IN pipe */
	while (!(Pipe_In_IsReceived()))
	{
		Pipe_SelectPipe(MASS_STORE_DATA_OUT_PIPE);

		/* Check if pipe stalled (command failed by device) */
		if (Pipe_IsStalled())
		  return false;

		Pipe_SelectPipe(MASS_STORE_DATA_IN_PIPE);

		/* Check if pipe stalled (command failed by device) */
		if (Pipe_IsStalled())
		  return false;
		
		UPINTX &= ~(1 << NAKEDI);
	};
	
	/* Retrieve the returned CSW */
	CommandStatusWrapper_t CSW;
	uint8_t*               CSWByte = (uint8_t*)&CSW;
	
	while (Pipe_BytesInPipe())
	  *(CSWByte++) = Pipe_Read_Byte();
	
	TEST_GLOBAL = CSW.Signature; // TEMP

	/* Clear the IN pipe, ready for next packet */
	Pipe_In_Clear();
	
	/* Return TRUE if command succeeded */
	return (CSW.Status == Command_Pass);
}

#if 0
bool MassStore_ReadDeviceBlock(uint32_t BlockAddress, uint8_t* BufferPtr)
{
	/* Set up counter for reading one block from device */
	uint16_t BytesRem = DEVICE_BLOCK_SIZE;

	/* Create a CBW with a SCSI command to read in the first two blocks from the device */
	CommandBlockWrapper_t SCSICommand =
		{
			Header:
				{
					Signature:          CBW_SIGNATURE,
					Tag:                0x00,
					DataTransferLength: BytesRem,
					Flags:              COMMAND_DIRECTION_DATA_IN,
					LUN:                0x00,
					SCSICommandLength:  10
				},
					
			SCSICommandData:
				{
					SCSI_CMD_READ_10,
					0x00,                   // Unused (control bits, all off)
					(BlockAddress >> 24),   // MSB of Block Address
					(BlockAddress >> 16),
					(BlockAddress >> 8),
					(BlockAddress & 0xFF),  // LSB of Block Address
					0x00,                   // Unused (reserved)
					0x00,                   // MSB of Total Blocks to Read
					0x01,                   // LSB of Total Blocks to Read
					0x00                    // Unused (control)
				}
		};
			
	uint8_t* CommandByte = (uint8_t*)&SCSICommand;
			
	/* Select the OUT data pipe for CBW transmission */
	Pipe_SelectPipe(MASS_STORE_DATA_OUT_PIPE);
		
	/* Write the CBW to the OUT pipe */
	for (uint8_t Byte = 0; Byte < sizeof(CommandBlockWrapper_t); Byte++)
	  Pipe_Write_Byte(*(CommandByte++));
	  
	/* Send the data in the OUT pipe to the attached device */
	Pipe_Out_Clear();

	/* Select the IN data pipe for data reception */
	Pipe_SelectPipe(MASS_STORE_DATA_IN_PIPE);
		
	/* Wait until data recieved in the IN pipe */
	while (!(Pipe_In_IsReceived()))
	{
		Pipe_SelectPipe(MASS_STORE_DATA_OUT_PIPE);

		/* Check if pipe stalled (command failed by device) */
		if (Pipe_IsStalled())
		  return false;

		Pipe_SelectPipe(MASS_STORE_DATA_IN_PIPE);

		/* Check if pipe stalled (command failed by device) */
		if (Pipe_IsStalled())
		  return false;
	};
	
	/* Loop until all bytes read */
	while (BytesRem)
	{
		/* Load each byte into the buffer */
		*(BufferPtr++) = Pipe_Read_Byte();
		
		/* When pipe is empty, clear it and wait for the next packet */
		if (!(Pipe_BytesInPipe()))
		{
			Pipe_In_Clear();
			while (!(Pipe_In_IsReceived()));
		}
	}
			
	/* Ignore the returned CSW */
	while (!(Pipe_In_IsReceived()));
	Pipe_In_Clear();

	return true;
}
#endif