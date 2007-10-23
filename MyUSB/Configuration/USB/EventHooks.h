/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the GPL Licence, Version 3
*/

#ifndef EVENTHOOKS_H
#define EVENTHOOKS_H

	/* Includes */
		#include "../../Common/Common.h"

	/* Private Macros */
		#define NO_EVENT_HOOK                    asm ("")

	/* Place any includes, defines and such used by your user-application hooks here. */
		#include "../../Drivers/USB1287/Serial_Stream.h"

	/* Configure user-application hooks here. */
		#define USB_EVENT_OnVBUSChange()	     MACROS{ printf("EVENT: VBus !=\r\n");    }MACROE
		#define USB_EVENT_OnVBUSConnect()        MACROS{ printf("EVENT: VBus +\r\n");     }MACROE
		#define USB_EVENT_OnVBUSDisconnect()     MACROS{ printf("EVENT: VBus -\r\n");     }MACROE
		#define USB_EVENT_OnUSBConnect()         MACROS{ printf("EVENT: USB  +\r\n");     }MACROE
		#define USB_EVENT_OnUSBDisconnect()      MACROS{ printf("EVENT: USB  -\r\n");     }MACROE
		#define USB_EVENT_OnSuspend()            MACROS{ printf("EVENT: USB Sleep\r\n");  }MACROE
		#define USB_EVENT_OnWakeUp()             MACROS{ printf("EVENT: USB Wakeup\r\n"); }MACROE
		#define USB_EVENT_OnReset()              MACROS{ printf("EVENT: USB Reset\r\n");  }MACROE
		#define USB_EVENT_OnUIDChange()          MACROS{ printf("EVENT: UID Change\r\n"); }MACROE
		#define USB_EVENT_PowerOnFail(errno)     MACROS{ printf("EVENT: Power on fail\r\n"); switch (USB_CurrentMode) { case USB_MODE_DEVICE: printf(" -- Mode DEVICE\r\n"); break; case USB_MODE_HOST: printf(" -- Mode HOST\r\n"); break; default: printf(" -- Mode N/A\r\n"); break; } printf(" -- Error Code %d\r\n", errno); }MACROE
		#define USB_EVENT_HostError(errno)       MACROS{ printf("EVENT: Host mode error\r\n"); printf(" -- Error Code %d\r\n", errno); }MACROE

#endif
