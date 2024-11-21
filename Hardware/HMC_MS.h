#ifndef _HMC_MS_H_
#define _HMC_MS_H_


void HMC5883L_WriteReg(uint8_t reg, uint8_t data);
uint8_t HMC5883L_ReadReg(uint8_t reg);
void HMC5883L_Init(void);
void HMC5883L_GetData(void);

void MS5611_Init(void);
void MS5611_Reset(void);
void MS5611_ReadPROM(void);
uint32_t MS5611_Do_Conversion(uint8_t command);
void MS5611_GetTemperature(u8 OSR_Temp);
void MS5611_GetData(void);
void Height_Update(float pressure);

#endif
