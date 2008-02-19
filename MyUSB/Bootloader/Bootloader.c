/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

/*
	MyUSB USB Bootloader. This bootloader enumerates to the host
	as a DFU Class device, allowing for DFU-compatible programming
	software to load firmware onto the AVR.
	
	This bootloader is compatible with Atmel's FLIP application.
	However, it requires the use of Atmel's DFU drivers. You will
	need to install Atmel's DFU drivers prior to using this bootloader.
	
	As an open-source option, this bootloader is also compatible
	with the Linux Atmel USB DFU Programmer software, avaliable
	for download at http://sourceforge.net/projects/dfu-programmer/.
	
    If SECURE_MODE is defined as true, upon startup the bootloader will
	be locked, with only the chip erase function avaliable (similar to
    Atmel's DFU bootloader). If SECURE_MODE is defined as false, all
	functions are usable on startup without the prerequisite firmware erase.
	
	NOTE: This device spoofs Atmel's DFU Bootloader USB VID and PID.
	      If you do not wish to use Atmel's codes, please manually
		  change them in Descriptors.c and alter your driver's INF
		  file accordingly.
    
	*** WORK IN PROGRESS - NOT CURRENTLY FUNCTIONING ***
*/

#define SECURE_MODE           false

#define INCLUDE_FROM_BOOTLOADER_C
#include "Bootloader.h"

bool          IsSecure      = SECURE_MODE;
bool          RunBootloader = true;

uint8_t       DFU_State     = dfuIDLE;
uint8_t       DFU_Status    = OK;

DFU_Command_t SentCommand;
uint8_t       ResponseByte;

FuncPtr_t     AppStartPtr   = 0x0000;

uint8_t       Flash64KBPage = 0;
uint16_t      StartAddr     = 0x0000;
uint16_t      EndAddr       = 0x0000;

uint8_t       SigBytes[3];

int main (void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	
	/* Relocate the interrupt vector table to the bootloader section */
	MCUCR = (1 << IVCE);
	MCUCR = (1 << IVSEL);

	/* Read in signature bytes from the AVR */
	SigBytes[0] = boot_read_sig_byte(0);
	SigBytes[1] = boot_read_sig_byte(2);
	SigBytes[2] = boot_read_sig_byte(4);

	/* Hardware initialization */
	Bicolour_Init();
	
	/* Staus LED set to red by default */
	Bicolour_SetLeds(BICOLOUR_LED1_RED);

	/* Initialize the USB subsystem */
	USB_Init();

	/* Run the USB management task while the bootloader is supposed to be running */
	while (RunBootloader)
	  USB_USBTask();
	
	/* Shut down the USB subsystem */
	USB_ShutDown();
	
	/* Relocate the interrupt vector table back to the application section */
	MCUCR = (1 << IVCE);
	MCUCR = 0;

	/* Reset any used hardware ports back to their defaults */
	PORTD = 0;
	DDRD  = 0;
	
	/* Start the user application */
	boot_rww_enable();
	AppStartPtr();
}

EVENT_HANDLER(USB_Connect)
{	
	/* Status LED green when USB connected */
	Bicolour_SetLeds(BICOLOUR_LED1_GREEN);
}

EVENT_HANDLER(USB_Disconnect)
{
	/* Upon disconnection, run user application */
	RunBootloader = false;
}

EVENT_HANDLER(USB_UnhandledControlPacket)
{
	uint8_t* CommandDataPtr = (uint8_t*)&SentCommand.Data;

	Endpoint_Ignore_Word();
	Endpoint_Ignore_Word();

	SentCommand.DataSize = Endpoint_Read_Word_LE();

	/* Processing request - status LEDs green/orange */
	Bicolour_SetLeds(BICOLOUR_LED1_GREEN | BICOLOUR_LED2_ORANGE);

	switch (Request)
	{
		case DFU_DNLOAD:
			Endpoint_ClearSetupReceived();
			
			if (DFU_State != dfuDNLOAD_IDLE)
			{
				/* If the request has a data stage, load it into the command struct */
				if (SentCommand.DataSize)
				{
					while (!(Endpoint_Setup_Out_IsReceived()));

					/* First byte of the data stage is the DNLOAD request's command */
					SentCommand.Command = Endpoint_Read_Byte();
					
					/* One byte of the data stage is the command, so subtract it from the total data bytes */
					SentCommand.DataSize--;
					
					/* Load in the rest of the data stage as command parameters */
					for (uint16_t DataByte = 0; DataByte < SentCommand.DataSize; DataByte++)
					  *(CommandDataPtr++) = Endpoint_Read_Byte();
					
					Endpoint_Setup_Out_Clear();
				}

				/* Process the command */
				ProcessBootloaderCommand();
			}
			else
			{
/* START TEST CODE */
				uint16_t TransfersRemaining = ((EndAddr - StartAddr) + 1);

				if (!(SentCommand.DataSize))
				{
					DFU_State = dfuIDLE;
				}
				else                                                           // Write FLASH or EEPROM
				{
					while (!(Endpoint_Setup_Out_IsReceived()));

					while (TransfersRemaining && SentCommand.DataSize)
					{
						if (IS_ONEBYTE_COMMAND(SentCommand.Data, 0x00))        // Write flash
						{
							uint32_t CurrFlashAddress = (((uint32_t)Flash64KBPage << 16) | StartAddr);

							boot_page_erase(CurrFlashAddress);
							boot_spm_busy_wait();
							
							for (uint16_t BytesInFlashPage = 0; ((BytesInFlashPage < SPM_PAGESIZE) &&
							      TransfersRemaining && SentCommand.DataSize); BytesInFlashPage += 2)
							{
								if (!(Endpoint_BytesInEndpoint()))
								{
									Endpoint_Setup_Out_Clear();
									while (!(Endpoint_Setup_Out_IsReceived()));
								}

								boot_page_fill((CurrFlashAddress + BytesInFlashPage), Endpoint_Read_Word_LE());	

								SentCommand.DataSize -= 2;
								TransfersRemaining   -= 2;
								StartAddr            += 2;
							}
								
							boot_page_write(CurrFlashAddress);
							boot_spm_busy_wait();
						}
						else                                                   // Write EEPROM
						{
							if (!(Endpoint_BytesInEndpoint()))
							{
								Endpoint_Setup_Out_Clear();
								while (!(Endpoint_Setup_Out_IsReceived()));
							}

							eeprom_write_byte((uint8_t*)StartAddr, Endpoint_Read_Byte());	

							SentCommand.DataSize    -= 1;
							TransfersRemaining      -= 1;
							StartAddr               += 1;
						}						
					}
				}
/* END TEST CODE */
			}

			/* Send ZLP to the host to acknowedge the request */
			Endpoint_Setup_In_Clear();
			while (!(Endpoint_Setup_In_IsReady()));
				
			break;
		case DFU_UPLOAD:
			Endpoint_ClearSetupReceived();

			while (!(Endpoint_Setup_In_IsReady()));

			if (DFU_State != dfuUPLOAD_IDLE)
			{
				/* Idle state upload - send response to last issued command */
				Endpoint_Write_Byte(ResponseByte);
			}
			else
			{
				uint16_t TransfersRemaining = ((EndAddr - StartAddr) + 1);

				if (IS_ONEBYTE_COMMAND(SentCommand.Data, 0x00) ||              // Read FLASH
				    IS_ONEBYTE_COMMAND(SentCommand.Data, 0x02))                // Read EEPROM
				{
					while (TransfersRemaining && SentCommand.DataSize)
					{
						/* Check if endpoint is empty - if so clear it and wait until ready for next packet */
						if (Endpoint_BytesInEndpoint() == ENDPOINT_CONTROLEP_SIZE)
						{
							Endpoint_Setup_In_Clear();
							while (!(Endpoint_Setup_In_IsReady()));
						}

						if (IS_ONEBYTE_COMMAND(SentCommand.Data, 0x00))        // Read FLASH
						{
							uint32_t CurrFlashAddress = (((uint32_t)Flash64KBPage << 16) | StartAddr);

							Endpoint_Write_Word_LE(pgm_read_word_far(CurrFlashAddress));

							SentCommand.DataSize -= 2;
							TransfersRemaining   -= 2;
							StartAddr            += 2;					
						}
						else                                                   // Read EEPROM
						{
							Endpoint_Write_Byte(eeprom_read_byte((uint8_t*)StartAddr));

							SentCommand.DataSize -= 1;
							TransfersRemaining   -= 1;
							StartAddr            += 1;					
						}
					}
				}
				else if (IS_ONEBYTE_COMMAND(SentCommand.Data, 0x01))           // Blank Check
				{
					/* Blank checking is performed in the DFU_DNLOAD request - if we get here we've told the host
					   that the memory isn't blank, and the host is requesting the first non-blank address */
					Endpoint_Write_Word_LE(StartAddr);
					
					/* Return to idle state */
					DFU_State = dfuIDLE;
				}
				
				/* Return to idle state if a short frame was sent (last packet of data) */
				if (!(TransfersRemaining))
				  DFU_State = dfuIDLE;
			}

			Endpoint_Setup_In_Clear();

			/* Send ZLP to the host to acknowedge the request */
			while (!(Endpoint_Setup_Out_IsReceived()));
			Endpoint_Setup_Out_Clear();

			break;
		case DFU_GETSTATUS:
			Endpoint_ClearSetupReceived();
			
			/* Write 8-bit status value */
			Endpoint_Write_Byte(DFU_Status);
			
			/* Write 24-bit poll timeout value */
			Endpoint_Write_Byte(0);
			Endpoint_Write_Word_LE(0);
			
			/* Write 8-bit state value */
			Endpoint_Write_Byte(DFU_State);

			/* Write 8-bit state string ID number */
			Endpoint_Write_Byte(0);

			Endpoint_Setup_In_Clear();
			
			while (!(Endpoint_Setup_Out_IsReceived()));
			Endpoint_Setup_Out_Clear();
	
			break;		
		case DFU_CLRSTATUS:
			Endpoint_ClearSetupReceived();
			
			/* Reset the status value variable to the default OK status */
			DFU_Status = OK;
			
			Endpoint_Setup_In_Clear();
			while (!(Endpoint_Setup_In_IsReady()));

			break;
		case DFU_GETSTATE:
			Endpoint_ClearSetupReceived();
			
			/* Write the current device state to the endpoint */
			Endpoint_Write_Byte(DFU_State);
		
			Endpoint_Setup_In_Clear();
			
			while (!(Endpoint_Setup_Out_IsReceived()));
			Endpoint_Setup_Out_Clear();

			break;
		case DFU_ABORT:
			Endpoint_ClearSetupReceived();
			
			/* Reset the current state variable to the default idle state */
			DFU_State = dfuIDLE;
			
			Endpoint_Setup_In_Clear();
			while (!(Endpoint_Setup_In_IsReady()));

			break;
	}

	/* Command processing finished, status LED green */
	Bicolour_SetLeds(BICOLOUR_LED1_GREEN);
}

static void ProcessBootloaderCommand(void)
{
	/* Check if device is in secure mode */
	if (IsSecure)
	{
		/* Don't process command unless it is a READ or chip erase command */
		if (!(((SentCommand.Command == COMMAND_WRITE)             &&
		        IS_TWOBYTE_COMMAND(SentCommand.Data, 0x00, 0xFF)) ||
			   (SentCommand.Command == COMMAND_READ)))
		{
			/* Set the state and status variables to indicate the error */
			DFU_State  = dfuERROR;
			DFU_Status = errWRITE;
			
			/* Stall command */
			Endpoint_StallTransaction();
			
			/* Don't process the command */
			return;
		}
	}

	/* Dispatch the required command processing routine based on the command type */
	switch (SentCommand.Command)
	{
		case COMMAND_WRITE:
			ProcessWriteCommand();
			break;
		case COMMAND_READ:
			ProcessReadCommand();
			break;
		case COMMAND_PROG_START:
			ProcessMemProgCommand();
			break;
		case COMMAND_DISP_DATA:
			ProcessMemReadCommand();
			break;
		case COMMAND_CHANGE_BASE_ADDR:
			if (IS_TWOBYTE_COMMAND(SentCommand.Data, 0x03, 0x00))              // Set 64KB flash page command
			  Flash64KBPage = SentCommand.Data[2];

			break;
	}
}

static void ProcessMemProgCommand(void)
{
	if (IS_ONEBYTE_COMMAND(SentCommand.Data, 0x00) ||                          // Write FLASH command
		IS_ONEBYTE_COMMAND(SentCommand.Data, 0x01))                            // Write EEPROM command
	{
		/* Load in the start and ending programming addresses */
		StartAddr = *((uint16_t*)&SentCommand.Data[1]);
		EndAddr   = *((uint16_t*)&SentCommand.Data[3]);

		DFU_State = dfuDNLOAD_IDLE;
	}
}

static void ProcessMemReadCommand(void)
{
	if (IS_ONEBYTE_COMMAND(SentCommand.Data, 0x00) ||                          // Read FLASH command
        IS_ONEBYTE_COMMAND(SentCommand.Data, 0x02))                            // Read EEPROM command
	{
		/* Load in the start and ending read addresses */
		StartAddr = *((uint16_t*)&SentCommand.Data[1]);
		EndAddr   = *((uint16_t*)&SentCommand.Data[3]);

		DFU_State = dfuUPLOAD_IDLE;
	}
	else if (IS_ONEBYTE_COMMAND(SentCommand.Data, 0x01))                       // Blank check FLASH command
	{
		uint32_t CurrFlashAddress = 0;

		while (CurrFlashAddress < BOOT_START_ADDR)
		{
			/* Check if the current byte is not blank */
			if (pgm_read_byte_far(CurrFlashAddress) != 0xFF)
			{
				/* Set state and status variables to the appropriate error values */
				DFU_State  = dfuERROR;
				DFU_Status = errCHECK_ERASED;

				break;
			}

			CurrFlashAddress++;
		}
	}
}

static void ProcessWriteCommand(void)
{
	if (IS_ONEBYTE_COMMAND(SentCommand.Data, 0x03))                            // Start application
	{
		/* Check if empty request data array - an empty request after a filled request retains the
		   previous valid request data, but initializes the reset */
		if (!(SentCommand.DataSize))
		{
			if (SentCommand.Data[1] == 0x00)                                   // Start via watchdog
			{
				/* Start the watchdog, enter infinite loop to reset the AVR */
				wdt_enable(WDTO_250MS);
				for (;;);
			}
			else                                                               // Start via jump
			{
				/* Load in the jump address into the application start address pointer */
				AppStartPtr = *(((FuncPtr_t*)&SentCommand.Data[3]));
				
				/* Set the flag to terminate the bootloader at next opportunity */
				RunBootloader = false;			
			}
		}
	}
	else if (IS_TWOBYTE_COMMAND(SentCommand.Data, 0x00, 0xFF))                 // Erase flash
	{
		uint32_t CurrFlashAddress = 0;

		/* Clear the application section of flash */
		while (CurrFlashAddress < BOOT_START_ADDR)
		{
			boot_page_erase(CurrFlashAddress);
			boot_spm_busy_wait();
			boot_page_write(CurrFlashAddress);
			boot_spm_busy_wait();

			CurrFlashAddress += SPM_PAGESIZE;
		}
					
		IsSecure = false;
	}
}

static void ProcessReadCommand(void)
{
	const uint8_t BootloaderInfo[3] = {BOOTLOADER_VERSION, BOOTLOADER_ID_BYTE1, BOOTLOADER_ID_BYTE2};

	uint8_t CommandResponse = 0x00;

	if (IS_ONEBYTE_COMMAND(SentCommand.Data, 0x00))                            // Read bootloader info
	{
		CommandResponse = BootloaderInfo[SentCommand.Data[1]];
	}
	else if (IS_ONEBYTE_COMMAND(SentCommand.Data, 0x01))                       // Read signature byte
	{
		if (SentCommand.Data[1] == 0x30)                                       // Read byte 1
		  CommandResponse = SigBytes[0];
		else if (SentCommand.Data[1] == 0x31)                                  // Read byte 2
		  CommandResponse = SigBytes[1];
		else if (SentCommand.Data[1] == 0x60)                                  // Read byte 3
		  CommandResponse = SigBytes[2];
	}
	
	ResponseByte = CommandResponse;
}
