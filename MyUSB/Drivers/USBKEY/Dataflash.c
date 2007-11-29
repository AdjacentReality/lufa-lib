/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the GPL Licence, Version 3
*/

#include "Dataflash.h"

void Dataflash_Init(void)
{
	SPCR  = ((1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0));
	SPSR  = (1 << SPI2X);
	
	DDRE |= DATAFLASH_CHIPCS_MASK;
	
	Dataflash_SelectChip(DATAFLASH_NO_CHIP);
}

void Dataflash_WaitWhileBusy(void)
{
	Dataflash_SendByte(DF_CMD_GETSTATUS);
	
	while (!(Dataflash_SendByte(0x00) & DF_STATUS_READY));
}

uint8_t Dataflash_SendByte(const uint8_t Byte)
{
	SPDR = Byte;
	while (!(SPSR & (1 << SPIF)));
	return SPDR;
}
