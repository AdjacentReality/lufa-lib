/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the GPL Licence, Version 3
*/

#ifndef _SCSI_H_
#define _SCSI_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/pgmspace.h>

		#include <MyUSB/Common/FunctionAttributes.h>  // Function Attributes
		#include <MyUSB/Drivers/USB/USB.h>            // USB Functionality
		#include <MyUSB/Drivers/USBKEY/Bicolour.h>    // Bicolour LEDs driver for the USBKEY

		#include "MassStorage.h"
		#include "Descriptors.h"
		#include "DataflashManager.h"
	
	/* Macros: */
		#define SCSI_SET_SENSE(key, acode, aqual)  		   MACROS{ SenseData.SenseKey = key;              \
		                                                           SenseData.AdditionalSenseCode = acode; \
		                                                           SenseData.AdditionalSenseQualifier = aqual; }MACROE

		#define DATA_READ                                  true
		#define DATA_WRITE                                 false
			
		#define SCSI_CMD_INQUIRY                           0x12
		#define SCSI_CMD_REQUEST_SENSE                     0x03
		#define SCSI_CMD_TEST_UNIT_READY                   0x00
		#define SCSI_CMD_READ_CAPACITY_10                  0x25
		#define SCSI_CMD_SEND_DIAGNOSTIC                   0x1D
		#define SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL      0x1E
		#define SCSI_CMD_WRITE_10                          0x2A
		#define SCSI_CMD_READ_10                           0x28
		#define SCSI_CMD_VERIFY_10                         0x2F
		#define SCSI_CMD_MODE_SENSE_6                      0x1A
		
		#define SCSI_SENSE_PAGE_READ_WRITE_ERR_RECOVERY    0x01
		#define SCSI_SENSE_PAGE_INFORMATIONAL_EXCEPTIONS   0x1C
		#define SCSI_SENSE_PAGE_ALL                        0x3F

		#define SCSI_SENSE_KEY_GOOD                        0x00
		#define SCSI_SENSE_KEY_RECOVERED_ERROR             0x01
		#define SCSI_SENSE_KEY_NOT_READY                   0x02
		#define SCSI_SENSE_KEY_MEDIUM_ERROR                0x03
		#define SCSI_SENSE_KEY_HARDWARE_ERROR              0x04
		#define SCSI_SENSE_KEY_ILLEGAL_REQUEST             0x05
		#define SCSI_SENSE_KEY_UNIT_ATTENTION              0x06
		#define SCSI_SENSE_KEY_DATA_PROTECT                0x07
		#define SCSI_SENSE_KEY_BLANK_CHECK                 0x08
		#define SCSI_SENSE_KEY_VENDOR_SPECIFIC             0x09
		#define SCSI_SENSE_KEY_COPY_ABORTED                0x0A
		#define SCSI_SENSE_KEY_ABORTED_COMMAND             0x0B
		#define SCSI_SENSE_KEY_VOLUME_OVERFLOW             0x0D
		#define SCSI_SENSE_KEY_MISCOMPARE                  0x0E

		#define SCSI_ASENSE_NO_ADDITIONAL_INFORMATION      0x00
		#define SCSI_ASENSE_LOGICAL_UNIT_NOT_READY         0x04
		#define SCSI_ASENSE_INVALID_FIELD_IN_CDB           0x24
		#define SCSI_ASENSE_WRITE_PROTECTED                0x27
		#define SCSI_ASENSE_FORMAT_ERROR                   0x31
		#define SCSI_ASENSE_INVALID_COMMAND                0x20
		#define SCSI_ASENSE_NOT_READY_TO_READY_CHANGE      0x28
		#define SCSI_ASENSE_MEDIUM_NOT_PRESENT             0x3A

		#define SCSI_ASENSEQ_NO_QUALIFIER                  0x00
		#define SCSI_ASENSEQ_FORMAT_COMMAND_FAILED         0x01
		#define SCSI_ASENSEQ_INITIALIZING_COMMAND_REQUIRED 0x02
		#define SCSI_ASENSEQ_OPERATION_IN_PROGRESS         0x07
		
	/* Type Defines: */
		typedef struct
		{
			unsigned int DeviceType          : 5;
			unsigned int PeripheralQualifier : 3;
			
			unsigned int _RESERVED1          : 7;
			unsigned int Removable           : 1;
			
			unsigned int ANSI_Version        : 3;
			unsigned int ECMA_Version        : 3;
			unsigned int ISO_IEC_Version     : 2;
			
			unsigned int ResponseDataFormat  : 4;
			unsigned int _RESERVED2          : 1;
			unsigned int NormACA             : 1;
			unsigned int TrmTsk              : 1;
			unsigned int AERC                : 1;

			uint8_t      AdditionalLength;
			uint8_t      _RESERVED3[2];

			unsigned int SoftReset           : 1;
			unsigned int CmdQue              : 1;
			unsigned int _RESERVED4          : 1;
			unsigned int Linked              : 1;
			unsigned int Sync                : 1;
			unsigned int WideBus16Bit        : 1;
			unsigned int WideBus32Bit        : 1;
			unsigned int RelAddr             : 1;
			
			uint8_t      VendorID[8];
			uint8_t      ProductID[16];
			uint8_t      RevisionID[4];
		} SCSI_Inquiry_Response_t;
		
		typedef struct
		{
			unsigned int ReponseCode         : 7;
			unsigned int Valid               : 1;
			
			uint8_t      SegmentNumber;
			
			unsigned int SenseKey            : 4;
			unsigned int _RESERVED1          : 1;
			unsigned int ILI                 : 1;
			unsigned int EOM                 : 1;
			unsigned int FileMark            : 1;
			
			uint8_t      Information[4];
			uint8_t      AdditionalLength;
			uint8_t      CmdSpecificInformation[4];
			uint8_t      AdditionalSenseCode;
			uint8_t      AdditionalSenseQualifier;
			uint8_t      FieldReplaceableUnitCode;
			uint8_t      SenseKeySpecific[3];
		} SCSI_Request_Sense_Response_t;
		
		typedef struct
		{
			unsigned int DCR  : 1;
			unsigned int DTE  : 1;
			unsigned int PER  : 1;
			unsigned int EER  : 1;
			unsigned int RC   : 1;
			unsigned int TB   : 1;
			unsigned int ARRE : 1;
			unsigned int AWRE : 1;

			uint8_t      ReadRetryCount;
			
			uint8_t      _RESERVED1[4];
			
			uint8_t      WriteRetryCount;
			
			uint8_t      _RESERVED2;

			uint16_t     RecoveryTimeLimit;
		} SCSI_Read_Write_Error_Recovery_Sense_Page_t;
		
		typedef struct
		{
			unsigned int LogErr      : 1;
			unsigned int _RESERVED1  : 1;
			unsigned int Test        : 1;
			unsigned int Dexcpt      : 1;
			unsigned int _RESERVED2  : 3;
			unsigned int Perf        : 1;
			
			unsigned int MRIE        : 4;
			unsigned int _RESERVED3  : 4;

			uint32_t     IntervalTimer;
		
			uint32_t     ReportCount;
		} SCSI_Informational_Exceptions_Sense_Page_t;
		
	/* Function Prototypes: */
		void SCSI_DecodeSCSICommand(void);
		
		#if defined(INCLUDE_FROM_SCSI_C)
			static bool SCSI_Command_Inquiry(void);
			static bool SCSI_Command_Request_Sense(void);
			static bool SCSI_Command_Read_Capacity_10(void);
			static bool SCSI_Command_Send_Diagnostic(void);
			static bool SCSI_Command_ReadWrite_10(const bool IsDataRead);
			static bool SCSI_Command_Mode_Sense_6(void);
			static void SCSI_WriteSensePage(const uint8_t PageCode, const uint8_t PageSize,
											const uint8_t* PageDataPtr,
			                                const int16_t AllocationLength)
			                                ATTR_NON_NULL_PTR_ARG(3);
		#endif
		
#endif
