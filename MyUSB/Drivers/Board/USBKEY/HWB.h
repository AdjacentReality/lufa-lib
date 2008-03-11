/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

#ifndef __HWB_USBKEY_H__
#define __HWB_USBKEY_H__

	/* Includes: */
		#include <avr/io.h>
		#include <stdbool.h>

		#include "../../../Common/Common.h"

	/* Preprocessor Checks: */
		#if !defined(INCLUDE_FROM_HWB_H)
			#error Do not include this file directly. Include MyUSB/Drivers/Board/HWB.h instead.
		#endif
		
	/* Public Interface - May be used in end-application: */
		/* Inline Functions: */
			static inline void HWB_Init(void)
			{
				DDRE  &= ~(1 << 2);
				PORTE |=  (1 << 2);
			}

			static inline bool HWB_GetStatus(void) ATTR_WARN_UNUSED_RESULT;
			static inline bool HWB_GetStatus(void)
			{
				return (!(PINE & (1 << 2)));
			}
			
#endif
