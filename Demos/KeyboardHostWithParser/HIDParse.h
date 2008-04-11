/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

#ifndef _HIDPARSE_H_
#define _HIDPARSE_H_

	/* Includes: */
		#include <avr/io.h>
		#include <string.h>
		
		#include <stdio.h>
		#include <avr/pgmspace.h>

		#include "HIDReportData.h"

	/* Macros: */
		#define HID_STACK_DEPTH      5
		#define HID_MAX_REPORTITEMS  30
		
	/* Enums: */
		enum HID_Types_t
		{
			REPORT_ITEM_TYPE_In               = 0,
			REPORT_ITEM_TYPE_Out              = 1,
			REPORT_ITEM_TYPE_Feature          = 2,
		};
		
		enum HID_Parse_ErrorCodes_t
		{
			HID_PARSE_Sucessful               = 0,
			HID_PARSE_StackOverflow           = 1,
			HID_PARSE_StackUnderflow          = 2,
			HID_PARSE_InsufficientReportItems = 3,
		};
	
	/* Type Defines: */
		typedef struct
		{
			uint32_t                     Minimum;
			uint32_t                     Maximum;
		} HID_MinMax_t;

		typedef struct
		{
			uint32_t                     Type;
			uint8_t                      Exponent;
		} HID_Unit_t;
		
		typedef struct
		{
			uint16_t                     Page;
			uint16_t                     Usage;
			uint16_t                     Minimum;
			uint16_t                     Maximum;
		} HID_Usage_t;

		typedef struct
		{
			uint8_t                      BitSize;
			
			HID_Usage_t                  Usage;
			HID_Unit_t                   Unit;
			HID_MinMax_t                 Logical;
			HID_MinMax_t                 Physical;	
		} HID_ReportItem_Attributes_t;
		
		typedef struct
		{
			uint16_t                     BitOffset;
			uint8_t                      ItemType;

			HID_ReportItem_Attributes_t  Attributes;
			
			uint32_t                     Value;
		} HID_ReportItem_t;
		
		typedef struct
		{
			 HID_ReportItem_Attributes_t Attributes;
			 uint8_t                     ReportCount;
		} HID_StateTable_t;

		typedef struct
		{
			uint8_t                      TotalReportItems;
			uint16_t                     BitOffsetIn;
			uint16_t                     BitOffsetOut;
			HID_ReportItem_t             ReportItems[HID_MAX_REPORTITEMS];
		} HID_ReportInfo_t;

	/* External Variables: */
		extern HID_ReportInfo_t   HIDReportInfo;

	/* Inline Functions: */
		static inline void ResetParser(void)
		{
			/* Reset the entire HID info structure */
			memset((void*)&HIDReportInfo, 0x00, sizeof(HID_ReportInfo_t)); 
		}
		
	/* Function Prototypes: */
		void    ResetParser(void);
		uint8_t ProcessHIDReport(uint8_t* ReportData, uint16_t ReportSize);

		void    GetReportItemInfo(uint8_t* ReportData, HID_ReportItem_t* ReportItem);
		void    SetReportItemInfo(uint8_t* ReportData, HID_ReportItem_t* ReportItem);

#endif
