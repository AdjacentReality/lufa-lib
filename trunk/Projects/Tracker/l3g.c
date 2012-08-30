#include "l3g.h"
#include "twi.h"

void l3g_init(void){

	unsigned char buf[6];

	//We will set 5 contiguous ctrl registers located at CTRL_REG1
	buf[0] = L3G_AUTO_INCREMENT | L3G_CTRL_REG1;
	
	buf[1] = L3G_DR_800HZ |	// Data rate
			 L3G_PD |			// Power on
			 L3G_ZEN |			// Z-Axis on
			 L3G_YEN |			// Y-Axis on 	
			 L3G_XEN;			// X-Axis on
	buf[2] = 0x00;					// CTRL_REG2
	buf[3] = L3G_H_LACTIVE |	// Active low
			 L3G_I2_DRDY;		// Enable DRDY interrupt pin on setting of ZYXDA (new data)
	buf[4] = L3G_FS;	// Scale selection
	buf[5] = 0x00;					// CTRL_REG5
	
	twi_write_block(L3G_ADDRESS, buf, 6);

}	

//Check for new data
unsigned char l3g_drdy(void){

	//The data available bit of the status reg acts just like the DRDY interrupt pin
	//It is set when new data is available, and cleared once the 6 bytes have been read
	return twi_write_then_read_byte(L3G_ADDRESS,L3G_STATUS_REG) & L3G_ZYXDA;

}

void l3g_read(short *x, short *y, short *z){

	unsigned char buf[6];
	buf[0] =  L3G_AUTO_INCREMENT | L3G_OUT_X_L;
	twi_write_then_read_block(L3G_ADDRESS,buf,1,buf, 6);

	*x = ((unsigned short)buf[1] << 8) | buf[0];
	*y = ((unsigned short)buf[3] << 8) | buf[2];
	*z = ((unsigned short)buf[5] << 8) | buf[4];

}
