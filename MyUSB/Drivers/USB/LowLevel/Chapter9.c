#include "Chapter9.h"

uint8_t USB_ConfigurationNumber;

void USB_ProcessControlPacket(void)
{
	uint8_t RequestType;
	uint8_t Request;
	
	RequestType = USB_Read_Byte();
	Request     = USB_Read_Byte();
			
	switch (Request)
	{
		case REQ_SetAddress:
			if (RequestType == 0x00000000)
			  USB_CHAP9_SetAddress();			
			break;
		case REQ_SetConfiguration:
			if (RequestType == 0b00000000)
			  USB_CHAP9_SetConfiguration();			
			break;
		case REQ_GetConfiguration:
			if (RequestType == 0b10000000)
			  USB_CHAP9_GetConfiguration();
			break;
		case REQ_GetDescriptor:
			if (RequestType == 0b10000000)
			  USB_CHAP9_GetDescriptor();
			break;
		default:		
			while (OCDR & (1 << IDRD)); // TEMP
			OCDR = 0; // TEMP
			while (OCDR & (1 << IDRD)); // TEMP
			OCDR = Request; // TEMP
			while (OCDR & (1 << IDRD)); // TEMP
			OCDR = RequestType; // TEMP
			while (OCDR & (1 << IDRD)); // TEMP
			OCDR = 0; // TEMP
	
			USB_ClearSetupRecieved();
			// TODO: Pass to user app?
			break;
	}
}

void USB_CHAP9_SetAddress(void)
{
	uint8_t NewAddress = USB_Read_Byte();

	UDADDR = ((UDADDR & (1 << ADDEN)) | (NewAddress & 0b00111111));

	USB_ClearSetupRecieved();

	USB_In_Clear();
	while (!(USB_In_IsReady()));
	
	UDADDR |= (1 << ADDEN);

	return;
}

void USB_CHAP9_SetConfiguration(void)
{
	uint8_t ConfigNum = USB_Read_Byte();
	
	if (ConfigNum <= CONFIGURATIONS)
	{
		USB_ConfigurationNumber = ConfigNum;
		USB_ClearSetupRecieved();
	}
	else
	{
		USB_Stall_Transaction();
		USB_ClearSetupRecieved();
		return;
	}
	
	USB_In_Clear();
	
	// TODO: User app endpoint configuration
	USB_EVENT_OnSetConfiguration();
}

void USB_CHAP9_GetConfiguration(void)
{
	USB_ClearSetupRecieved();
	
	USB_Write_Byte(USB_ConfigurationNumber);
	
	USB_In_Clear();

	while(!(USB_Out_IsRecieved()));
	USB_Out_Clear();
}

void USB_CHAP9_GetDescriptor(void)
{
	uint8_t  DescriptorIndex = USB_Read_Byte();
	volatile uint8_t  DescriptorType  = USB_Read_Byte(); // TEMP volatile
	volatile uint16_t DescriptorLength; // TEMP volatile
	
	void*    DescriptorPointer;
	uint16_t DescriptorBytesRem;
	
	bool     SendZeroPacket  = false;
	
	switch (DescriptorType)
	{
		case DTYPE_Device:
			DescriptorPointer  = (void*)&DeviceDescriptor;
			DescriptorBytesRem = sizeof(DeviceDescriptor);
			break;
		case DTYPE_Configuration:
			DescriptorPointer  = (void*)&ConfigurationDescriptor;
			DescriptorBytesRem = sizeof(ConfigurationDescriptor);
			break;			
		default:
			if (USB_GetDescriptorString(DescriptorIndex, &DescriptorPointer, &DescriptorBytesRem) == false)
			{
				USB_Stall_Transaction();
				USB_ClearSetupRecieved();
				return;
			}
	}

	USB_Ignore_Word(); // Ignore language identifier

	DescriptorLength  = USB_Read_Word();
	
	USB_ClearSetupRecieved();
	
	if (DescriptorLength > DescriptorBytesRem)
	{
		if ((DescriptorBytesRem % ENDPOINT_CONTROLEP_SIZE) == 0)
		  SendZeroPacket = true;
	}
	else
	{
		DescriptorBytesRem = DescriptorLength;
	}
	
	while (DescriptorBytesRem && (!(USB_Out_IsRecieved()))) // New packet aborts transfer
	{
		while (!(USB_In_IsReady()));
		
		uint8_t BytesInPacket = 0;
		
		while (DescriptorBytesRem && (BytesInPacket++ < ENDPOINT_CONTROLEP_SIZE)) // Fill the control endpoint
		{
			USB_Write_Byte(pgm_read_byte_near(DescriptorPointer++));
			DescriptorBytesRem--;
		}
		
		USB_In_Clear();
	}
	
	USB_In_Clear();

	if (USB_Out_IsRecieved()) // Host abort
	{
		USB_Out_Clear();
		return;
	}
	
	if(SendZeroPacket == true)
	{
		while (!(USB_In_IsReady()));
		USB_In_Clear();
	}

   while(!(USB_Out_IsRecieved()));
   USB_Out_Clear();
}
