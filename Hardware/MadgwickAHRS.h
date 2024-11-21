#ifndef _MADGWICKAHRS_H_
#define _MADGWICKAHRS_H_


//extern const float PI, RAD_TO_DEGREE, DEGREE_TO_RAD, RAW_TO_RAD;
extern volatile float q0, q1, q2, q3;
extern volatile float beta;
extern const float RAD_TO_DEGREE, DEGREE_TO_RAD, RAW_TO_RAD;

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
//void TIM2_Init(void);

#endif
