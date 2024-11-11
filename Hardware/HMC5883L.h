#ifndef _HMC5883L_H_
#define _HMC5883L_H_



void HMC5883L_WriteReg(uint8_t reg, uint8_t data);

uint8_t HMC5883L_ReadReg(uint8_t reg);

void HMC5883L_Init(void);

void HMC5883L_GetData(void);


#endif
