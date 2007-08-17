/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the GPL Licence, Version 3
*/

#include "Serial.h"

void Serial_Init(const uint16_t BaudRate)
{
	UCSR1A = 0;
	UCSR1B = ((1 << RXEN1) | (1 << TXEN1));
	UCSR1C = ((1 << UCSZ11) | (1 << UCSZ11));
	
	UBRR1  = SERIAL_UBBRVAL(BaudRate);
}

void Serial_TxString_P(const char *FlashStringPtr)
{
	uint8_t CurrByte;

	while ((CurrByte = pgm_read_byte(FlashStringPtr)) != 0x00)
	{
		Serial_Tx(CurrByte);
		FlashStringPtr++;
	}
}

void Serial_TxString(const char *StringPtr)
{
	while (*StringPtr != 0x00)
	{
		Serial_Tx(*StringPtr);
		StringPtr++;
	}
}

void Serial_Tx(const char Data)
{
	while (!(UCSRA & (1 << UDRE)));
	UDR = Data;
}

char Serial_Rx(void)
{
	while (!(UCSR1A & (1 << RXC1)));
	return UDR1; 
}
