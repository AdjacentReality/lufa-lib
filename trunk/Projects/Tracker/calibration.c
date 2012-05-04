/* Calibration for Tracker's Magnetometer */

#include <avr/eeprom.h>
#include <inttypes.h>

// locations for data we're storing in EEPROM
uint8_t EEMEM nv_cal_check;
float EEMEM nv_ox;
float EEMEM nv_oy;
float EEMEM nv_oz;
float EEMEM nv_sx;
float EEMEM nv_sy;
float EEMEM nv_sz;

// use the default scale values for the LSM devices
float ox = 0.0;
float oy = 0.0;
float oz = 0.0;
float sx = 1100.0;
float sy = 1100.0;
float sz = 980.0;

void calibration_load(void)
{
    if (eeprom_read_byte(&nv_cal_check) == 'c') {
        ox = eeprom_read_float(&nv_ox);
        oy = eeprom_read_float(&nv_oy);
        oz = eeprom_read_float(&nv_oz);
        sx = eeprom_read_float(&nv_sx);
        sy = eeprom_read_float(&nv_sy);
        sz = eeprom_read_float(&nv_sz);
    }
}

void calibration_store(float nox, float noy, float noz, float nsx, float nsy, float nsz)
{
    ox = nox;
    eeprom_update_float(&nv_ox, ox);
    
    oy = noy;
    eeprom_update_float(&nv_oy, oy);
    
    oz = noz;
    eeprom_update_float(&nv_oz, oz);
    
    sx = nsx;
    eeprom_update_float(&nv_sx, sx);
    
    sy = nsy;
    eeprom_update_float(&nv_sy, sy);
    
    sz = nsz;
    eeprom_update_float(&nv_sz, sz);
    
    eeprom_update_byte(&nv_cal_check, 'c');
}

// Just correcting the hard iron offset and sensor scaling
void calibration_apply(short rawx, float *x, short rawy, float *y, float rawz, float *z)
{
    *x = ((float)rawx-ox)/sx;
    *y = ((float)rawy-oy)/sy;
    *z = ((float)rawz-oz)/sz;
}

