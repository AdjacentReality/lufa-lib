/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

#ifndef __HWB_H__
#define __HWB_H__

	/* Macros: */
	#define INCLUDE_FROM_HWB_H
	#define INCLUDE_FROM_BOARD_DRIVER

	/* Includes: */
	#include "../../Common/BoardTypes.h"
	
	#if defined(BOARD) && (BOARD == BOARD_USBKEY)
		#include "USBKEY/HWB.h"
	#elif defined(BOARD) && (BOARD == BOARD_STK525)
		#include "STK525/HWB.h"
	#elif defined(BOARD) && (BOARD == BOARD_STK526)
		#include "STK526/HWB.h"
	#else
		#error BOARD must be set in makefile to BOARD_USBKEY, BOARD_STK525 or BOARD_STK526.
	#endif

#endif
