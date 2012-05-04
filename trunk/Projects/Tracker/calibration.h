#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

void calibration_load(void);

void calibration_store(float nox, float noy, float noz, float nsx, float nsy, float nsz);

void calibration_apply(short rawx, float *x, short rawy, float *y, float rawz, float *z);

#endif /* _CALIBRATION_H_ */
