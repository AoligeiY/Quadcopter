#include "MPU6050_Reg.h"
#include "includes.h"

extern OS_EVENT* IICMutex;			// IIC锁
extern int16_t Acel[3], Gyro[3], Mag[3];

/**
	MPU6050向寄存器写数据
**/
void MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
	IIC_Start();
	IIC_SendByte((MPU6050_Address)<<1|0);
	IIC_ReceiveACK();
	IIC_SendByte(reg);
	IIC_ReceiveACK();
	
	IIC_SendByte(data);
	IIC_ReceiveACK();
	IIC_Stop();
}

/**
	MPU6050读寄存器数据
**/
uint8_t MPU6050_ReadReg(uint8_t reg)
{
	uint8_t data;
	IIC_Start();
	IIC_SendByte((MPU6050_Address)<<1|0);
	IIC_ReceiveACK();
	IIC_SendByte(reg);
	IIC_ReceiveACK();
	
	IIC_Start();
	IIC_SendByte((MPU6050_Address)<<1|1);
	IIC_ReceiveACK();
	data = IIC_ReceiveByte();
	IIC_SendACK(1);
//	IIC_SendACK(0);
	IIC_Stop();
	
	return data;
}

void MPU6050_Init(void)
{
	int i = 0, j = 0;
    //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
    for (i = 0; i < 1000; i++) {
        for (j = 0; j < 1000; j++) {
            ;
        }
    }
	
//	if (MPU6050_GetID() != MPU6050_Device_ID) {
//		return 0;
//	}
	
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x00);		//解除休眠状态,使用内部8MHz振荡器
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x00);		//采样分频 (采样频率 = 陀螺仪输出频率 / (1+DIV)，采样频率1000hz）
	MPU6050_WriteReg(MPU6050_CONFIG,0x06);			//设置低通滤波
	// 11月1日：0x18->0x00, 量程修改为+-250度
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x00);		//陀螺仪满量程+-2000度/秒 (最低分辨率 = 2^15/2000 = 16.4LSB/度/秒
	// 11月1日：0x08->0x00, 0x00量程为+-2G
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x00);	//加速度满量程+-4g   (最低分辨率 = 2^15/4g = 8192LSB/g )
	
//	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01);
//	MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);
//	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x09);
//	MPU6050_WriteReg(MPU6050_CONFIG,0x06);
//	MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x18);
//	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x18);
	
	MPU6050_WriteReg(MPU6050_INT_PIN_CFG, 0x02);		//开启BYPASS模式（辅助IIC），使HMC5883L的总线直接与STM32的总线接口物理连接
	MPU6050_WriteReg(MPU6050_USER_CTRL, 0x00);			//关闭主模式
	
//	return 1;
}

// 获取MPU6050ID号，为0x68
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}


/**
	读取MPU6050加速度计陀螺仪xyz轴数据
**/
void MPU6050_GetData(void)
{
	INT8U err;
	uint8_t DataH,DataL;
	OSMutexPend(IICMutex, 0, &err);
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	Acel[0] = (DataH << 8)| DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	Acel[1] = (DataH << 8)| DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	Acel[2] = (DataH << 8)| DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	Gyro[0] = (DataH << 8)| DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	Gyro[1] = (DataH << 8)| DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	Gyro[2] = (DataH << 8)| DataL;
	
	OSMutexPost(IICMutex);
	
}

//void MPU6050_GetData(void)
//{
//	INT8U err;
//	//uint还是int
//	int8_t DataH,DataL;
//	OSMutexPend(IICMutex, 0, &err);
//	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
//	Acc_Gyro.AccX = (int16_t)(DataH<<8)|DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
//	Acc_Gyro.AccY = (int16_t)(DataH<<8)|DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
//	Acc_Gyro.AccZ = (int16_t)(DataH<<8)|DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
//	Acc_Gyro.GyroX = (int16_t)(DataH<<8)|DataL;
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
//	Acc_Gyro.GyroY = (int16_t)(DataH<<8)|DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
//	Acc_Gyro.GyroZ = (int16_t)(DataH<<8)|DataL;
//	
//	
//	
//	OSMutexPost(IICMutex);
//	
//	fGyro.x = (float)(Acc_Gyro.GyroX * RAW_TO_RAD);
//	fGyro.y = (float)(Acc_Gyro.GyroY * RAW_TO_RAD);
//	fGyro.z = (float)(Acc_Gyro.GyroZ * RAW_TO_RAD);
////	printf("Acc_Gyro.GyroX: %d",Acc_Gyro.GyroX);
////	printf("fGyro.z = %f",fGyro.z);
//	
//	
//}

// 已弃用
// 以下为发送原始加速度计陀螺仪数据给匿名解析
// 数据帧格式不对且数据不经过任何处理，改用Send_QuaBUFF发送四元数数据帧
void Send_BUFF(void)
{
	//MPU6050_GetData(&Data1);
	
	uint8_t i;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt=0;							
	int8_t BUFF[30];
	
	BUFF[_cnt++] = 0xAA;
	BUFF[_cnt++] = 0xFF;
	BUFF[_cnt++] = 0x01;
	BUFF[_cnt++] = 13;
	BUFF[_cnt++] = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	BUFF[_cnt++] = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	BUFF[_cnt++] = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	BUFF[_cnt++] = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	BUFF[_cnt++] = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	BUFF[_cnt++] = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	BUFF[_cnt++] = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	BUFF[_cnt++] = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	BUFF[_cnt++] = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	BUFF[_cnt++] = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	BUFF[_cnt++] = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	BUFF[_cnt++] = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
//	BUFF[_cnt++] = BYTE0(Data1.AccX);
//	BUFF[_cnt++] = BYTE1(Data1.AccX);
//	BUFF[_cnt++] = BYTE0(Data1.AccY);
//	BUFF[_cnt++] = BYTE1(Data1.AccY);
//	BUFF[_cnt++] = BYTE0(Data1.AccZ);
//	BUFF[_cnt++] = BYTE1(Data1.AccZ);
//	BUFF[_cnt++] = BYTE0(Data1.GyroX);
//	BUFF[_cnt++] = BYTE1(Data1.GyroX);
//	BUFF[_cnt++] = BYTE0(Data1.GyroY);
//	BUFF[_cnt++] = BYTE1(Data1.GyroY);
//	BUFF[_cnt++] = BYTE0(Data1.GyroZ);
//	BUFF[_cnt++] = BYTE1(Data1.GyroZ);
	BUFF[_cnt++] = 0;
	
	for(i=0;i<BUFF[3]+4;i++) 
	{
		sumcheck+=BUFF[i];
		addcheck+=sumcheck;
	}
	BUFF[_cnt++]=sumcheck;	
	BUFF[_cnt++]=addcheck;
	
	for(i=0;i<_cnt;i++)
	{
		Serial_SendByte(BUFF[i]);
	}
}	

//void toEulerAngle(Angle *angle){
//	angle->yaw = atan2(2 * q1 * q2  + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3* q3 + 1) * RAD_TO_DEGREE; 					// yaw
//	angle->pitch = asin(-2 * q1 * q3 + 2 * q0* q2) * RAD_TO_DEGREE - pitchOffset; 							    	// pitch
//	angle->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * RAD_TO_DEGREE - rollOffset; 	// roll
//}


