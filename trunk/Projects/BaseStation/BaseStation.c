/*
             LUFA Library
     Copyright (C) Dean Camera, 2012.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2012  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the Tracker Base Station.
 */

#include "BaseStation.h"
#include "version.h"
#include "led.h"
#include "packet.h"
#include "uart.h"
#include "gpio.h"

/** LED mask for the library LED driver, to indicate that the USB interface is not ready. */
static const unsigned char LEDMASK_USB_NOTREADY[3] = {255, 0, 0};

/** LED mask for the library LED driver, to indicate that the USB interface is enumerating. */
static const unsigned char LEDMASK_USB_ENUMERATING[3] = {255, 255, 0};

/** LED mask for the library LED driver, to indicate that the USB interface is ready. */
static const unsigned char LEDMASK_USB_READY[3] = {0, 255, 0};

/** LED mask for the library LED driver, to indicate that an error has occurred in the USB interface. */
static const unsigned char LEDMASK_USB_ERROR[3] = {255, 0, 0};

// Default to streaming nothing
static uint8_t streaming_mode = 0;
#define SHOULD_STREAM(m) (streaming_mode & (1 << m))

static bool demo_mode = true;

static bool useUSB = 0;

uint32_t Boot_Key ATTR_NO_INIT;
#define MAGIC_BOOT_KEY 0xDC42ACCA
#define BOOTLOADER_START_ADDRESS ((unsigned int)((32-4)*1024))

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t Tracker_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = 0,
				.DataINEndpoint                 =
					{
						.Address                = CDC_TX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.DataOUTEndpoint                =
					{
						.Address                = CDC_RX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.NotificationEndpoint           =
					{
						.Address                = CDC_NOTIFICATION_EPADDR,
						.Size                   = CDC_NOTIFICATION_EPSIZE,
						.Banks                  = 1,
					},
			},
	};

/** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevHIDReportBuffer[TRACKER_REPORT_SIZE];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Tracker_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = 2,
				.ReportINEndpoint             =
					{
						.Address              = TRACKER_EPADDR,
						.Size                 = TRACKER_EPSIZE,
						.Banks                = 1,
					},
				.PrevReportINBuffer           = PrevHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevHIDReportBuffer),
			},
	};

// Jump to bootloader from software.  From the LUFA docs.
void BootloaderJumpCheck(void) ATTR_INIT_SECTION(3);
void BootloaderJumpCheck(void)
{
    // If the reset source was the bootloader and the key is correct, clear it and jump to the bootloader
    if ((MCUSR & (1 << WDRF)) && (Boot_Key == MAGIC_BOOT_KEY))
    {
        Boot_Key = 0;
        ((void (*)(void))BOOTLOADER_START_ADDRESS)();
    }
}

void JumpToBootloader(void)
{
    // If USB is used, detach from the bus and reset it
    USB_Disable();
    // Disable all interrupts
    cli();
    // Wait two seconds for the USB detachment to register on the host
    Delay_MS(2000);
    // Set the bootloader key to the magic value and force a reset
    Boot_Key = MAGIC_BOOT_KEY;
    wdt_enable(WDTO_250MS);
    for (;;);
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
	
	/* Hardware Initialization */
	led_init();
//	gpio_init();
	USB_Init();
	uart_init(38400, false);
	
	// Set the 16 bit timer to increment at F_CPU/1024 Hz
	TCCR1B = 0x05;
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	led_set_array(LEDMASK_USB_NOTREADY);
	sei();

    // Reset the timer so the setup time isn't taken into account
    TCNT1 = 0;
	for (;;)
	{
	    ReadData();
		SendData();
        
        /* Must throw away unused bytes from the host, or it will lock up while waiting for the device */
        if (!useUSB) // we otherwise grab bytes in ReadData if we are using USB
    		CDC_Device_ReceiveByte(&Tracker_CDC_Interface);

		CDC_Device_USBTask(&Tracker_CDC_Interface);
		HID_Device_USBTask(&Tracker_HID_Interface);
		USB_USBTask();
	}
}

static void ParseByte(unsigned char c)
{
    packet_t p;
    // use the packets as they complete
    if (packet_unpack(&p, c)) {
        // exit demo mode if we get a packet
        demo_mode = false;
        switch(p.type) {
            // TODO: the rest of the packet types
        
            case PACKET_COLOR:
                led_set_array(p.data.color);
                break;
                
            case PACKET_IR:
                led_set_ir(p.data.bitmask);
                break;
                
            case PACKET_STREAM:
                streaming_mode = p.data.bitmask;
                break;
                
            case PACKET_GPIO_DDR:
                gpio_set_ddr(p.data.bitmask);
                break;
                
            case PACKET_GPIO_PORT:
                gpio_set_port(p.data.bitmask);
                break;
                
            case PACKET_BOOTLOAD:
                led_set_colors(127, 0, 0);
                JumpToBootloader();
                break;
        }
    }
}

void ReadData(void)
{
    if (useUSB) {
        int to_read = CDC_Device_BytesReceived(&Tracker_CDC_Interface);
        if (to_read > 0) {
            while (to_read--) {
                int16_t in_byte = CDC_Device_ReceiveByte(&Tracker_CDC_Interface);
                if (in_byte >= 0)
                    ParseByte(in_byte);
                else
                    break; // stop if we fail to read a byte
            }
        }
    } else {
        while (1) {
            int16_t in_byte = uart_getc();
            if (in_byte >= 0)
                ParseByte(in_byte);
            else
                break;
        }
    }
}

#define GYRO_TO_RADIANS(a) ((float)a*M_PI*L3G_SO/(180.0*1000.0))

short g[3], m[3], a[3];
float gf[3], mf[3];

static void PackAndSend(packet_p p)
{
    unsigned char buf[PACKET_MAX_SIZE];
    int packed_size = packet_pack(p, buf);
    if (useUSB)
        CDC_Device_SendData(&Tracker_CDC_Interface, (const char const *)buf, packed_size);
    else
        uart_send(buf, packed_size);
}

void SendData(void)
{
    packet_t p;
    
    if (SHOULD_STREAM(PACKET_ACC)) {
        p.type = PACKET_ACC;
        p.data.sensor[0] = a[0];
        p.data.sensor[1] = a[1];
        p.data.sensor[2] = a[2];
        PackAndSend(&p);
    }
    
    if (SHOULD_STREAM(PACKET_GYRO)) {
        p.type = PACKET_GYRO;
        p.data.sensor[0] = g[0];
        p.data.sensor[1] = g[1];
        p.data.sensor[2] = g[2];
        PackAndSend(&p);
    }
    
    if (SHOULD_STREAM(PACKET_MAG)) {
        p.type = PACKET_MAG;
        p.data.sensor[0] = m[0];
        p.data.sensor[1] = m[1];
        p.data.sensor[2] = m[2];
        PackAndSend(&p);
    }
    
    // TODO: temperature support
    
    if (SHOULD_STREAM(PACKET_GPIO)) {
        p.type = PACKET_GPIO;
        p.data.bitmask = gpio_pin();
        PackAndSend(&p);
    }
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	led_set_array(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	led_set_array(LEDMASK_USB_NOTREADY);
	useUSB = 0;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&Tracker_CDC_Interface);
	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Tracker_HID_Interface);

	USB_Device_EnableSOFEvents();

	led_set_array(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
	useUSB = ConfigSuccess;
    if (ConfigSuccess) demo_mode = false;
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&Tracker_CDC_Interface);
	HID_Device_ProcessControlRequest(&Tracker_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Tracker_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean true to force the sending of the report, false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	short *sData = (short *)ReportData;
	unsigned char *bData = (unsigned char *)ReportData;
    
    // Both avr-libc and USB are little endian, so this is a no-op.
    sData[0] = cpu_to_le16(g[0]);
    sData[1] = cpu_to_le16(g[1]);
    sData[2] = cpu_to_le16(g[2]);
    // Can get away with not doing endianness conversion here, but remember to do it on other platforms.
    // The accelerometer is 12-bit high aligned, magnetometer is 12 bit low aligned
    bData[6] = (a[0] >> 8);
    bData[7] = (a[0] & 0xF0) | ((a[1] >> 12) & 0x0F);
    bData[8] = (a[1] >> 4);
    bData[9] = (a[2] >> 8);
    bData[10] = (a[2] & 0xF0) | ((m[0] >> 8) & 0x0F);
    bData[11] = m[0];
    bData[12] = m[1] >> 4;
    bData[13] = (m[1] << 4) | ((m[2] >> 8) & 0x0F);
    bData[14] = m[2];
    
	*ReportSize = TRACKER_REPORT_SIZE;
	return false;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
	// Unused (but mandatory for the HID class driver) in this demo, since there are no Host->Device reports
}

