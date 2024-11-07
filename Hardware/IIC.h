#ifndef __IIC_H_
#define __IIC_H_

void IIC_W_SCL(uint8_t value);
void IIC_W_SDA(uint8_t value);
uint8_t IIC_R_SDA(void);
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_SendByte(uint8_t Byte);
uint8_t IIC_ReceiveByte(void);
void IIC_SendACK(uint8_t AckBit);
uint8_t IIC_ReceiveACK(void);


#endif
