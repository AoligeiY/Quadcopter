#ifndef _MPU6050_H_
#define _MPU6050_H_

#define	ACC_RANGE	4
#define	GYRO_RANGE	500

typedef struct Angle{
	float yaw, pitch, roll;
}Angle;

void MPU6050_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU6050_MPUReadReg(uint8_t reg);
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(void);
void Send_BUFF(void);
//void toEulerAngle(Angle *angle);

#endif
