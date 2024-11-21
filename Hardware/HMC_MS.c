#include "HMC_MS_Reg.h"
#include "includes.h"

extern OS_EVENT* IICMutex;
extern int16_t Acel[3], Gyro[3], Mag[3];

extern float Pressure;			//温度补偿大气压
extern float Temperature; 		//实际温度
extern float Height;
extern float offsetPress;

uint16_t Cal_C[7]; 				//用于存放PROM中的6组数据
uint32_t D1_Pres, D2_Temp; 		// 存放数字压力和温度
float dT, Temperature2; 		//实际和参考温度之间的差异,中间值
double OFF, SENS; 				//实际温度抵消,实际温度灵敏度
float Aux, OFF2, SENS2; 		//温度校验值


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
	HMC5883L_WriteReg(HMC5883L_Config_A,0x78);	//11月1日：0x70->0x18	11.16:0x78->0x18
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
	
	if (Mag[0] > 0x7fff){
		Mag[0] -= 0xffff;
	}
	if (Mag[1] > 0x7fff){
		Mag[1] -= 0xffff;
	}
	if (Mag[2] > 0x7fff){
		Mag[2] -= 0xffff;
	}
	
	OSMutexPost(IICMutex);
}


// 气压计初始化
void MS5611_Init(void)
{
	MS5611_Reset();
	Delay_ms(100);
	MS5611_ReadPROM();
	Delay_ms(100);
}

// 气压计复位
void MS5611_Reset(void)
{
	IIC_Start();
	IIC_SendByte(MS561101BA_Addr);
	IIC_SendByte(MS561101BA_RESET);
	IIC_Stop();
}

// 从PROM读取出厂校准数据
void MS5611_ReadPROM(void)
{
    u8 i;
    for (i = 0; i <= MS561101BA_PROM_REG_COUNT; i++) {
        Cal_C[i] = IIC_Read_2Bytes(MS561101BA_Addr, MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
    }
}

uint32_t MS5611_Do_Conversion(uint8_t command)
{
    uint32_t conversion;
    INT8U err;
	
	OSMutexPend(IICMutex, 0, &err);
	IIC_Start();
    IIC_SendByte(MS561101BA_Addr);
    IIC_SendByte(command);
    IIC_Stop();
	OSMutexPost(IICMutex);
	
	OSTimeDly(9);
	
	OSMutexPend(IICMutex, 0, &err);
	conversion = IIC_Read_3Bytes(MS561101BA_Addr, 0);
	OSMutexPost(IICMutex);
	
	return conversion;
}	


// 读取数字温度
void MS5611_GetTemperature(u8 OSR_Temp)
{
    D2_Temp = MS5611_Do_Conversion(OSR_Temp);

    OSTimeDly(9);

    dT = D2_Temp - (((uint32_t)Cal_C[5]) << 8);
    Temperature = 2000 + dT * ((uint32_t)Cal_C[6]) / 0x800000; //算出温度值的100倍，2001表示20.01°
}

// 读取数字气压
void MS5611_GetPressure(u8 OSR_Pres)
{

    D1_Pres = MS5611_Do_Conversion(OSR_Pres);

    OSTimeDly(9);

    OFF = (uint32_t)(Cal_C[2] << 16) + ((uint32_t)Cal_C[4] * dT) / 0x80;
    SENS = (uint32_t)(Cal_C[1] << 15) + ((uint32_t)Cal_C[3] * dT) / 0x100;
    //温度补偿
    if (Temperature < 2000) // second order temperature compensation when under 20 degrees C
    {
        Temperature2 = (dT * dT) / 0x80000000;
        Aux = (Temperature - 2000) * (Temperature - 2000);
        OFF2 = 2.5f * Aux;
        SENS2 = 1.25f * Aux;
        if (Temperature < -1500) {
            Aux = (Temperature + 1500) * (Temperature + 1500);
            OFF2 = OFF2 + 7 * Aux;
            SENS2 = SENS + 5.5f * Aux;
        }
    } else //(Temperature > 2000)
    {
        Temperature2 = 0;
        OFF2 = 0;
        SENS2 = 0;
    }

    Temperature = Temperature - Temperature2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;

    Pressure = (D1_Pres * SENS / 0x200000 - OFF) / 0x8000;
}

void MS5611_GetData(void) {
	MS5611_GetTemperature(MS561101BA_D2_OSR_4096); 	// 0x58
	MS5611_GetPressure(MS561101BA_D1_OSR_4096);		// 0x48
}
	
void Height_Update(float pressure) {
	float baro_height = (pressure - offsetPress) * K_PRESS_TO_HIGH * 100; 		//气压计相对高度cm
    Height = (Height * 7 + baro_height * 3) / 10;
}
