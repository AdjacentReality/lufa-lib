/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

#ifndef __BICOLOUR_H__
#define __BICOLOUR_H__

	/* Macros: */
	#define INCLUDE_FROM_BICOLOUR_H
	#define INCLUDE_FROM_BOARD_DRIVER

	/* Includes: */
	#include "../../Common/BoardTypes.h"
	
	#if defined(BOARD) && (BOARD == BOARD_USBKEY)
		#include "USBKEY/Bicolour.h"
	#elif defined(BOARD) && (BOARD == BOARD_STK525)
		#include "STK525/Bicolour.h"
	#elif defined(BOARD) && (BOARD == BOARD_STK526)
		#include "STK526/Bicolour.h"
	#else
		#error BOARD must be set in makefile to BOARD_USBKEY, BOARD_STK525 or BOARD_STK526.
	#endif

#endif
