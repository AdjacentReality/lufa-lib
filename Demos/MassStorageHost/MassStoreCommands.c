/*
             LUFA Library
     Copyright (C) Dean Camera, 2009.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com
*/

/*
  Copyright 2009  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, and distribute this software
  and its documentation for any purpose and without fee is hereby
  granted, provided that the above copyright notice appear in all
  copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#define  INCLUDE_FROM_MASSSTORE_COMMANDS_C
#include "MassStoreCommands.h"

/* Globals: */
CommandBlockWrapper_t  SCSICommandBlock;
CommandStatusWrapper_t SCSICommandStatus;
uint32_t               MassStore_Tag = 1;

static uint8_t MassStore_SendCommand(void)
{
	uint8_t ErrorCode = PIPE_RWSTREAM_ERROR_NoError;

	/* Each transmission should have a unique tag value, excluding valued 0 and 0xFFFFFFFF */
	if (++MassStore_Tag == 0xFFFFFFFF)
	  MassStore_Tag = 1;

	/* Select the OUT data pipe for CBW transmission */
	Pipe_SelectPipe(MASS_STORE_DATA_OUT_PIPE);
	Pipe_Unfreeze();

	/* Write the CBW command to the OUT pipe */
	if ((ErrorCode = Pipe_Write_Stream_LE(&SCSICommandBlock, sizeof(CommandBlockWrapper_t))) != PIPE_RWSTREAM_ERROR_NoError)
	  return ErrorCode;

	/* Send the data in the OUT pipe to the attached device */
	Pipe_ClearCurrentBank();

	/* Freeze pipe after use */
	Pipe_Freeze();
	
	return PIPE_RWSTREAM_ERROR_NoError;
}

static uint8_t MassStore_WaitForDataReceived(void)
{
	uint16_t TimeoutMSRem = COMMAND_DATA_TIMEOUT_MS;

	/* Unfreeze the OUT pipe so that it can be checked */
	Pipe_SelectPipe(MASS_STORE_DATA_OUT_PIPE);
	Pipe_Unfreeze();

	/* Select the IN data pipe for data reception */
	Pipe_SelectPipe(MASS_STORE_DATA_IN_PIPE);
	Pipe_Unfreeze();

	/* Wait until data received in the IN pipe */
	while (!(Pipe_ReadWriteAllowed()))
	{
		/* Check to see if a new frame has been issued (1ms elapsed) */
		if (USB_INT_HasOccurred(USB_INT_HSOFI))
		{
			/* Clear the flag and decrement the timout period counter */
			USB_INT_Clear(USB_INT_HSOFI);
			TimeoutMSRem--;

			/* Check to see if the timeout period for the command has elapsed */
			if (!(TimeoutMSRem))
			  return PIPE_RWSTREAM_ERROR_Timeout;
		}
	
		Pipe_SelectPipe(MASS_STORE_DATA_OUT_PIPE);

		/* Check if pipe stalled (command failed by device) */
		if (Pipe_IsStalled())
		{
			/* Clear the stall condition on the OUT pipe */
			MassStore_ClearPipeStall(MASS_STORE_DATA_OUT_PIPE);

			return PIPE_RWSTREAM_ERROR_PipeStalled;
		}

		Pipe_SelectPipe(MASS_STORE_DATA_IN_PIPE);

		/* Check if pipe stalled (command failed by device) */
		if (Pipe_IsStalled())
		{
			/* Clear the stall condition on the IN pipe */
			MassStore_ClearPipeStall(MASS_STORE_DATA_IN_PIPE);

			return PIPE_RWSTREAM_ERROR_PipeStalled;
		}
		  
		/* Check to see if the device was disconnected, if so exit function */
		if (!(USB_IsConnected))
		  return PIPE_RWSTREAM_ERROR_DeviceDisconnected;
	};

	return PIPE_RWSTREAM_ERROR_NoError;
}

static uint8_t MassStore_SendReceiveData(void* BufferPtr)
{
	uint8_t  ErrorCode = PIPE_RWSTREAM_ERROR_NoError;
	uint16_t BytesRem  = SCSICommandBlock.Header.DataTransferLength;

	/* Check the direction of the SCSI command data stage */
	if (SCSICommandBlock.Header.Flags & COMMAND_DIRECTION_DATA_IN)
	{
		/* Select the IN data pipe for data reception */
		Pipe_SelectPipe(MASS_STORE_DATA_IN_PIPE);
		Pipe_Unfreeze();
		
		/* Read in the block data from the pipe */
		if ((ErrorCode = Pipe_Read_Stream_LE(BufferPtr, BytesRem)) != PIPE_RWSTREAM_ERROR_NoError)
		  return ErrorCode;
	}
	else
	{
		/* Select the OUT data pipe for data transmission */
		Pipe_SelectPipe(MASS_STORE_DATA_OUT_PIPE);
		Pipe_Unfreeze();

		/* Write the block data to the pipe */
		if ((ErrorCode = Pipe_Write_Stream_LE(BufferPtr, BytesRem)) != PIPE_RWSTREAM_ERROR_NoError)
		  return ErrorCode;
	}
	
	/* Acknowledge the packet */
	Pipe_ClearCurrentBank();

	/* Freeze used pipe after use */
	Pipe_Freeze();

	return PIPE_RWSTREAM_ERROR_NoError;
}

static uint8_t MassStore_GetReturnedStatus(void)
{
	uint8_t ErrorCode = PIPE_RWSTREAM_ERROR_NoError;

	/* Select the IN data pipe for data reception */
	Pipe_SelectPipe(MASS_STORE_DATA_IN_PIPE);
	Pipe_Unfreeze();
	
	/* Load in the CSW from the attached device */
	if ((ErrorCode = Pipe_Read_Stream_LE(&SCSICommandStatus, sizeof(CommandStatusWrapper_t))) != PIPE_RWSTREAM_ERROR_NoError)
	  return ErrorCode;
	  
	/* Clear the data ready for next reception */
	Pipe_ClearCurrentBank();

	/* Freeze the IN pipe after use */
	Pipe_Freeze();
	
	return PIPE_RWSTREAM_ERROR_NoError;
}

uint8_t MassStore_ClearPipeStall(const uint8_t PipeEndpointNum)
{
	USB_HostRequest = (USB_Host_Request_Header_t)
		{
			bmRequestType: (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_ENDPOINT),
			bRequest:      REQ_ClearFeature,
			wValue:        FEATURE_ENDPOINT_HALT,
			wIndex:        PipeEndpointNum,
			wLength:       0,
		};
	
	return USB_Host_SendControlRequest(NULL);
}

uint8_t MassStore_MassStorageReset(void)
{
	USB_HostRequest = (USB_Host_Request_Header_t)
		{
			bmRequestType: (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE),
			bRequest:      MASS_STORAGE_RESET,
			wValue:        0,
			wIndex:        0,
			wLength:       0,
		};
	
	return USB_Host_SendControlRequest(NULL);
}

uint8_t MassStore_GetMaxLUN(uint8_t* const MaxLUNIndex)
{
	uint8_t ErrorCode;

	USB_HostRequest = (USB_Host_Request_Header_t)
		{
			bmRequestType: (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE),
			bRequest:      GET_MAX_LUN,
			wValue:        0,
			wIndex:        0,
			wLength:       1,
		};
		
	if ((ErrorCode = USB_Host_SendControlRequest(MaxLUNIndex)) == HOST_SENDCONTROL_SetupStalled)
	{
		/* Clear the pipe stall */
		Pipe_ClearStall();
	
		/* Some faulty Mass Storage devices don't implement the GET_MAX_LUN request, so assume a single LUN */
		*MaxLUNIndex = 0;	
	}
	
	return ErrorCode;
}

uint8_t MassStore_RequestSense(const uint8_t LUNIndex, const SCSI_Request_Sense_Response_t* const SensePtr)
{
	uint8_t ReturnCode = PIPE_RWSTREAM_ERROR_NoError;

	/* Create a CBW with a SCSI command to issue REQUEST SENSE command */
	SCSICommandBlock = (CommandBlockWrapper_t)
		{
			Header:
				{
					Signature:          CBW_SIGNATURE,
					Tag:                MassStore_Tag,
					DataTransferLength: sizeof(SCSI_Request_Sense_Response_t),
					Flags:              COMMAND_DIRECTION_DATA_IN,
					LUN:                LUNIndex,
					SCSICommandLength:  6
				},
					
			SCSICommandData:
				{
					SCSI_CMD_REQUEST_SENSE,
					0x00,                   // Reserved
					0x00,                   // Reserved
					0x00,                   // Reserved
					sizeof(SCSI_Request_Sense_Response_t), // Allocation Length
					0x00                    // Unused (control)
				}
		};
	
	/* Send SCSI command to the attached device */
	MassStore_SendCommand();

	/* Wait until data received from the device */
	if ((ReturnCode = MassStore_WaitForDataReceived()) != PIPE_RWSTREAM_ERROR_NoError)
	{
		Pipe_Freeze();
		return ReturnCode;
	}

	/* Read the returned sense data into the buffer */
	if ((ReturnCode = MassStore_SendReceiveData((uint8_t*)SensePtr)) != PIPE_RWSTREAM_ERROR_NoError)
	{
		Pipe_Freeze();
		return ReturnCode;
	}	
	
	/* Read in the returned CSW from the device */
	if ((ReturnCode = MassStore_GetReturnedStatus()) != PIPE_RWSTREAM_ERROR_NoError)
	{
		Pipe_Freeze();
		return ReturnCode;
	}
	
	return PIPE_RWSTREAM_ERROR_NoError;
}

uint8_t MassStore_ReadDeviceBlock(const uint8_t LUNIndex, const uint32_t BlockAddress,
                                  const uint8_t Blocks, const uint16_t BlockSize, void* BufferPtr)
{
	uint8_t ReturnCode = PIPE_RWSTREAM_ERROR_NoError;

	/* Create a CBW with a SCSI command to read in the given blocks from the device */
	SCSICommandBlock = (CommandBlockWrapper_t)
		{
			Header:
				{
					Signature:          CBW_SIGNATURE,
					Tag:                MassStore_Tag,
					DataTransferLength: ((uint32_t)Blocks * BlockSize),
					Flags:              COMMAND_DIRECTION_DATA_IN,
					LUN:                LUNIndex,
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
					Blocks,                 // LSB of Total Blocks to Read
					0x00                    // Unused (control)
				}
		};
	
	/* Send SCSI command to the attached device */
	MassStore_SendCommand();

	/* Wait until data received from the device */
	if ((ReturnCode = MassStore_WaitForDataReceived()) != PIPE_RWSTREAM_ERROR_NoError)
	{
		Pipe_Freeze();
		return ReturnCode;
	}

	/* Read the returned block data into the buffer */
	if ((ReturnCode = MassStore_SendReceiveData(BufferPtr)) != PIPE_RWSTREAM_ERROR_NoError)
	{
		Pipe_Freeze();
		return ReturnCode;
	}	
	
	/* Read in the returned CSW from the device */
	if ((ReturnCode = MassStore_GetReturnedStatus()) != PIPE_RWSTREAM_ERROR_NoError)
	{
		Pipe_Freeze();
		return ReturnCode;
	}
	
	return PIPE_RWSTREAM_ERROR_NoError;
}

uint8_t MassStore_WriteDeviceBlock(const uint8_t LUNIndex, const uint32_t BlockAddress,
                                   const uint8_t Blocks, const uint16_t BlockSize, void* BufferPtr)
{
	uint8_t ReturnCode = PIPE_RWSTREAM_ERROR_NoError;

	/* Create a CBW with a SCSI command to write the given blocks to the device */
	SCSICommandBlock = (CommandBlockWrapper_t)
		{
			Header:
				{
					Signature:          CBW_SIGNATURE,
					Tag:                MassStore_Tag,
					DataTransferLength: ((uint32_t)Blocks * BlockSize),
					Flags:              COMMAND_DIRECTION_DATA_OUT,
					LUN:                LUNIndex,
					SCSICommandLength:  10
				},
					
			SCSICommandData:
				{
					SCSI_CMD_WRITE_10,
					0x00,                   // Unused (control bits, all off)
					(BlockAddress >> 24),   // MSB of Block Address
					(BlockAddress >> 16),
					(BlockAddress >> 8),
					(BlockAddress & 0xFF),  // LSB of Block Address
					0x00,                   // Unused (reserved)
					0x00,                   // MSB of Total Blocks to Write
					Blocks,                 // LSB of Total Blocks to Write
					0x00                    // Unused (control)
				}
		};
	
	/* Send SCSI command to the attached device */
	MassStore_SendCommand();

	/* Write the data to the device from the buffer */
	if ((ReturnCode = MassStore_SendReceiveData(BufferPtr)) != PIPE_RWSTREAM_ERROR_NoError)
	{
		Pipe_Freeze();
		return ReturnCode;
	}	
	
	/* Read in the returned CSW from the device */
	if ((ReturnCode = MassStore_GetReturnedStatus()) != PIPE_RWSTREAM_ERROR_NoError)
	{
		Pipe_Freeze();
		return ReturnCode;
	}
	
	return PIPE_RWSTREAM_ERROR_NoError;
}

uint8_t MassStore_TestUnitReady(const uint8_t LUNIndex)
{
	uint8_t ReturnCode = PIPE_RWSTREAM_ERROR_NoError;	

	/* Create a CBW with a SCSI command to issue TEST UNIT READY command */
	SCSICommandBlock = (CommandBlockWrapper_t)
		{
			Header:
				{
					Signature:          CBW_SIGNATURE,
					Tag:                MassStore_Tag,
					DataTransferLength: 0,
					Flags:              COMMAND_DIRECTION_DATA_IN,
					LUN:                LUNIndex,
					SCSICommandLength:  6
				},
					
			SCSICommandData:
				{
					SCSI_CMD_TEST_UNIT_READY,
					0x00,                   // Reserved
					0x00,                   // Reserved
					0x00,                   // Reserved
					0x00,                   // Reserved
					0x00                    // Unused (control)
				}
		};
	
	/* Send SCSI command to the attached device */
	MassStore_SendCommand();

	/* Read in the returned CSW from the device */
	if ((ReturnCode = MassStore_GetReturnedStatus()) != PIPE_RWSTREAM_ERROR_NoError)
	{
		Pipe_Freeze();
		return ReturnCode;
	}
	
	return PIPE_RWSTREAM_ERROR_NoError;
}

uint8_t MassStore_ReadCapacity(const uint8_t LUNIndex, SCSI_Capacity_t* const CapacityPtr)
{
	uint8_t ReturnCode = PIPE_RWSTREAM_ERROR_NoError;

	/* Create a CBW with a SCSI command to issue READ CAPACITY command */
	SCSICommandBlock = (CommandBlockWrapper_t)
		{
			Header:
				{
					Signature:          CBW_SIGNATURE,
					Tag:                MassStore_Tag,
					DataTransferLength: 8,
					Flags:              COMMAND_DIRECTION_DATA_IN,
					LUN:                LUNIndex,
					SCSICommandLength:  10
				},
					
			SCSICommandData:
				{
					SCSI_CMD_READ_CAPACITY_10,
					0x00,                   // Reserved
					0x00,                   // MSB of Logical block address
					0x00,
					0x00,
					0x00,                   // LSB of Logical block address
					0x00,                   // Reserved
					0x00,                   // Reserved
					0x00,                   // Partial Medium Indicator
					0x00                    // Unused (control)
				}
		};
	
	/* Send SCSI command to the attached device */
	MassStore_SendCommand();

	/* Wait until data received from the device */
	if ((ReturnCode = MassStore_WaitForDataReceived()) != PIPE_RWSTREAM_ERROR_NoError)
	{
		Pipe_Freeze();
		return ReturnCode;
	}

	/* Read the returned capacity data into the buffer */
	if ((ReturnCode = MassStore_SendReceiveData(CapacityPtr)) != PIPE_RWSTREAM_ERROR_NoError)
	{
		Pipe_Freeze();
		return ReturnCode;
	}
	  
	/* Endian-correct the read data */
	CapacityPtr->Blocks    = SwapEndian_32(CapacityPtr->Blocks);
	CapacityPtr->BlockSize = SwapEndian_32(CapacityPtr->BlockSize);
	
	/* Read in the returned CSW from the device */
	if ((ReturnCode = MassStore_GetReturnedStatus()) != PIPE_RWSTREAM_ERROR_NoError)
	{
		Pipe_Freeze();
		return ReturnCode;
	}
	
	return PIPE_RWSTREAM_ERROR_NoError;
}

uint8_t MassStore_PreventAllowMediumRemoval(const uint8_t LUNIndex, const bool PreventRemoval)
{
	uint8_t ReturnCode = PIPE_RWSTREAM_ERROR_NoError;

	/* Create a CBW with a SCSI command to issue PREVENT ALLOW MEDIUM REMOVAL command */
	SCSICommandBlock = (CommandBlockWrapper_t)
		{
			Header:
				{
					Signature:          CBW_SIGNATURE,
					Tag:                MassStore_Tag,
					DataTransferLength: 0,
					Flags:              COMMAND_DIRECTION_DATA_OUT,
					LUN:                LUNIndex,
					SCSICommandLength:  6
				},
					
			SCSICommandData:
				{
					SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL,
					0x00,                   // Reserved
					0x00,                   // Reserved
					PreventRemoval,         // Prevent flag
					0x00,                   // Reserved
					0x00                    // Unused (control)
				}
		};
	
	/* Send SCSI command to the attached device */
	MassStore_SendCommand();

	/* Read in the returned CSW from the device */
	if ((ReturnCode = MassStore_GetReturnedStatus()) != PIPE_RWSTREAM_ERROR_NoError)
	{
		Pipe_Freeze();
		return ReturnCode;
	}
	
	return PIPE_RWSTREAM_ERROR_NoError;
}
