#ifndef __CALIBRATION_H_
#define __CALIBRATION_H_

typedef struct cali{
	float Ox, Sx, Oy, Sy, Oz, Sz;
}cali;

extern cali accCali;
extern cali gyroCali;
extern cali magCali;
extern uint8_t gyroOffset, accOffset, pressOffset;
extern float K_PRESS_TO_HIGH;

void Open_Calib(void);
u8 Calib_Status(void);
void MPU6050_Offset(void);
//void PrepareData(void);

#endif
