#ifndef __CALIBRATION_H_
#define __CALIBRATION_H_

typedef struct cali{
	float Ox, Sx, Oy, Sy, Oz, Sz;
}cali;

extern cali accCali;
extern cali gyroCali;
extern cali magCali;

//void PrepareData(void);

#endif
