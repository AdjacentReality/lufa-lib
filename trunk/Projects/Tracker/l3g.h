/*
    Interface and definitions for configuring and reading from ST's L3G line
    of digital 3-axis gyroscopes.  Tested at least partially with L3G4200D
    and L3GD20.  Others should work with little to no additional configuration.
*/
    
#ifndef L3G_H
#define L3G_H

// Value of WHO_AM_I. Can be incremented by 2 if the SA0 line is pulled up
#define L3G4300D    0xD0 // or D2
#define L3GD20      0xD4 // or D6

// Change this to whichever device you're using
// TODO: add automagic detection of which device is connected
#define L3G_ADDRESS	    L3GD20

void l3g_init(void);
unsigned char l3g_drdy(void);
void l3g_read(short *x, short *y, short *z);

// Register Address Map
#define L3G_CTRL_REG1              0x20
#define L3G_CTRL_REG2              0x21
#define L3G_CTRL_REG3              0x22
#define L3G_CTRL_REG4              0x23
#define L3G_CTRL_REG5              0x24
#define L3G_STATUS_REG             0x27
#define L3G_OUT_X_L                0x28	

// The register addresses are all 0x7F masked
// For multibyte read/write the address can be set to auto increment via the MSB
#define L3G_AUTO_INCREMENT         0x80

//CTRL_REG1: Data rates
#define L3G_DR_100HZ		0x00
#define L3G_DR_200HZ		0x40
#define L3G_DR_400HZ		0x80
#define L3G_DR_800HZ		0xC0
//CTRL_REG1: Enables
#define L3G_PD			    0x08
#define L3G_ZEN			    0x04
#define L3G_YEN 		    0x02
#define L3G_XEN			    0x01
//CTRL_REG3: Interrupts	
#define L3G_H_LACTIVE 	    0x20
#define L3G_I2_DRDY		    0x08
//CTRL_REG4: Scale
#define L3G_FS_250DPS		0x00
#define L3G_FS_500DPS		0x10
#define L3G_FS_2000DPS		0x30
//STATUS_REG
#define L3G_ZYXDA			0x08
#define L3G_ZYXOR			0x80

#endif
