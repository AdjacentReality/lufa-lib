#ifndef TEMPERATURE_H
#define TEMPERATURE_H

	/* Includes */
		#include <avr/pgmspace.h>

		#include "../USB1287/ADC.h"
		#include "../../Common/FunctionAttributes.h"

	/* Public Macros */
		#define TEMP_ADC_CHANNEL   0
		#define TEMP_MIN_TEMP      TEMP_TABLE_OFFSET
		#define TEMP_MAX_TEMP      ((TEMP_TABLE_SIZE - 1) + TEMP_TABLE_OFFSET)
		
		#define Temperature_Init() ADC_SetupChannel(ADC_TEMP_CHANNEL);

	/* Private Macros */
		#define TEMP_TABLE_SIZE   (sizeof(Temperature_Lookup) / sizeof(Temperature_Lookup[0]))
		#define TEMP_TABLE_OFFSET -20		

	/* Function Prototypes */
		int8_t Temperature_GetTemperature(void) ATTR_WARN_UNUSED_RESULT;
		
#endif
