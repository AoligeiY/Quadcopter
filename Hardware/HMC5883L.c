#include "HMC5883L_Reg.h"
#include "includes.h"

extern OS_EVENT* IICMutex;
extern int16_t Acel[3], Gyro[3], Mag[3];

/**
	HMC5883L向寄存器写数据
**/
void HMC5883L_WriteReg(uint8_t reg, uint8_t data)
{
	IIC_Start();
	IIC_SendByte((HMC5883L_Address)<<1|0);
	IIC_ReceiveACK();
	IIC_SendByte(reg);
	IIC_ReceiveACK();
	
	IIC_SendByte(data);
	IIC_ReceiveACK();
	IIC_Stop();
}

/**
	HMC5883L读寄存器数据
**/
uint8_t HMC5883L_ReadReg(uint8_t reg)
{
	uint8_t data;
	IIC_Start();
	IIC_SendByte((HMC5883L_Address)<<1|0);
	IIC_ReceiveACK();
	IIC_SendByte(reg);
	IIC_ReceiveACK();
	
	IIC_Start();
	IIC_SendByte((HMC5883L_Address)<<1|1);
	IIC_ReceiveACK();
	data = IIC_ReceiveByte();
	IIC_SendACK(1);
//	IIC_SendACK(0);
	IIC_Stop();
	
	return data;
}

void HMC5883L_Init(void)
{
	// 设置标准数据输出速率75HZ
	HMC5883L_WriteReg(HMC5883L_Config_A,0x78);	//11月1日：0x70->0x18
	// 设置传感器磁场范围±1.3Ga
	HMC5883L_WriteReg(HMC5883L_Config_B,0x20);
	// 打开continuous measurement模式
	HMC5883L_WriteReg(HMC5883L_Mode,0x00);
	
}


/**
	读取HMC5883L磁力计xyz轴数据
**/
void HMC5883L_GetData(void)
{
	INT8U err;
	uint8_t DataH,DataL;
	OSMutexPend(IICMutex, 0, &err);
	
	DataH = HMC5883L_ReadReg(HMC5883L_MAGNETIC_XOUT_H);
	DataL = HMC5883L_ReadReg(HMC5883L_MAGNETIC_XOUT_L);
	Mag[0] = (DataH<<8)|DataL;
	
	DataH = HMC5883L_ReadReg(HMC5883L_MAGNETIC_YOUT_H);
	DataL = HMC5883L_ReadReg(HMC5883L_MAGNETIC_YOUT_L);
	Mag[1] = (DataH<<8)|DataL;
	
	DataH = HMC5883L_ReadReg(HMC5883L_MAGNETIC_ZOUT_H);
	DataL = HMC5883L_ReadReg(HMC5883L_MAGNETIC_ZOUT_L);
	Mag[2] = (DataH<<8)|DataL;
	
	OSMutexPost(IICMutex);
}


//void HMC5883L_GetData(void)
//{
//	INT8U err;
//	uint8_t DataH,DataL;
//	OSMutexPend(IICMutex, 0, &err);
//	
//	DataH = HMC5883L_ReadReg(HMC5883L_MAGNETIC_XOUT_H);
//	DataL = HMC5883L_ReadReg(HMC5883L_MAGNETIC_XOUT_L);
//	Mag.MagX = (int16_t)(DataH<<8)|DataL;
//	
//	DataH = HMC5883L_ReadReg(HMC5883L_MAGNETIC_YOUT_H);
//	DataL = HMC5883L_ReadReg(HMC5883L_MAGNETIC_YOUT_L);
//	Mag.MagY = (int16_t)(DataH<<8)|DataL;
//	
//	DataH = HMC5883L_ReadReg(HMC5883L_MAGNETIC_ZOUT_H);
//	DataL = HMC5883L_ReadReg(HMC5883L_MAGNETIC_ZOUT_L);
//	Mag.MagZ = (int16_t)(DataH<<8)|DataL;
//	
//	OSMutexPost(IICMutex);
//}
