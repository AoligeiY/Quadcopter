#include "stm32f4xx.h"                  // Device header
#include "Delay.h"

void IIC_W_SCL(uint8_t value)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_2, (BitAction)value);
//	OSTimeDly(1);
//	OSSchedLock();
//	OSTimeDlyHMSM(0, 0, 0, 555);
//	OSSchedUnlock();
	Delay_us(5);
}

void IIC_W_SDA(uint8_t value)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_3, (BitAction)value);
//	OSTimeDly(1);
//	OSSchedLock();
//	OSTimeDlyHMSM(0, 0, 0, 555);
//	OSSchedUnlock();
	Delay_us(5);
}

uint8_t IIC_R_SDA(void)
{
	uint8_t value;
	value = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3);
//	OSTimeDly(1);
//	OSSchedLock();
//	OSTimeDlyHMSM(0, 0, 0, 555);
//	OSSchedUnlock();
	Delay_us(5);
	return value;
}

void IIC_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOC,GPIO_Pin_2|GPIO_Pin_3);
}

void IIC_Start(void)
{
	IIC_W_SDA(1);
	IIC_W_SCL(1);
	IIC_W_SDA(0);
	IIC_W_SCL(0);
}

void IIC_Stop(void)
{
	IIC_W_SDA(0);
	IIC_W_SCL(1);
	IIC_W_SDA(1);
}

void IIC_SendByte(uint8_t Byte)
{
	uint8_t i;
	for(i = 0; i<8; i++)
	{
		IIC_W_SDA(Byte&(0x80>>i));
		IIC_W_SCL(1);
		IIC_W_SCL(0);
	}
}

uint8_t IIC_ReceiveByte(void)
{
	uint8_t i;
	uint8_t Byte = 0x00;
	IIC_W_SDA(1);
	
	for(i = 0; i<8; i++)
	{
		IIC_W_SCL(1);
		if(IIC_R_SDA() == 1)
		{
			Byte = Byte|(0x80>>i);
		}
		IIC_W_SCL(0);
	}
	return Byte;
}

void IIC_SendACK(uint8_t AckBit)
{
	IIC_W_SDA(AckBit);
	IIC_W_SCL(1);
	IIC_W_SCL(0);
}

uint8_t IIC_ReceiveACK(void)
{
	uint8_t AckBit;
	IIC_W_SDA(1);
	
	IIC_W_SCL(1);
	AckBit = IIC_R_SDA();
	IIC_W_SCL(0);
	
	return AckBit;
}

