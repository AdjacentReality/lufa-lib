/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the GPL Licence, Version 3
*/

#ifndef __DATAFLASH_H__
#define __DATAFLASH_H__

	/* Includes: */
		#include <avr/io.h>
		
		#include "../../Common/Common.h"
		#include "../Misc/DataflashCommandBytes.h"
		
	/* Public Interface - May be used in end-application: */
		/* Macros: */
			#define DATAFLASH_NO_CHIP            DATAFLASH_CHIPCS_MASK
			#define DATAFLASH_CHIP1              (1 << 1)
			#define DATAFLASH_CHIP2              (1 << 0)
					
			#define Dataflash_GetSelectedChip()         (PORTE & DATAFLASH_CHIPCS_MASK)
			#define Dataflash_SelectChip(mask)   MACROS{ PORTE = ((PORTE & ~DATAFLASH_CHIPCS_MASK) | mask); }MACROE
			#define Dataflash_DeselectChip()     Dataflash_SelectChip(DATAFLASH_CHIPCS_MASK)

		/* Function Prototypes: */
			void    Dataflash_Init(void);
			void    Dataflash_WaitWhileBusy(void);
			uint8_t Dataflash_SendByte(const uint8_t Byte);
			
	/* Private Interface - For use in library only: */
		/* Macros */
			#define DATAFLASH_CHIPCS_MASK        (DATAFLASH_CHIP1 | DATAFLASH_CHIP2)

#endif
