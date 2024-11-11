#ifndef _MPU6050_H_
#define _MPU6050_H_

//#define RAW_TO_RAD 0.0010653f	//原始数据转弧度

#define	ACC_RANGE	4
#define	GYRO_RANGE	500


void MPU6050_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU6050_MPUReadReg(uint8_t reg);
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(void);
//void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
//						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void Send_BUFF(void);
#endif
