#include "stm32f4xx.h"                  // Device header
#include "Delay.h"
#include <stdio.h>
#include <stdarg.h>
#include "Serial.h"
#include "includes.h"

uint16_t Serial_RxData;		//接收数据
uint8_t Serial_RxFlag;		//接收标志位


#if 1

/*	
	以下部分将关闭标准库半主机模式，以在不使用MicroLib的情况下实现printf接口重定向串口
*/

/* 告知连接器不从C库链接使用半主机的函数 */
#pragma import(__use_no_semihosting)
 
/* 定义 _sys_exit() 以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}
 
/* 标准库需要的支持类型 */
struct __FILE
{
    int handle;
};
 
FILE __stdout;

int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

//重定义fputc函数
//int fputc(int ch, FILE *f)
//{
//    while((USART1->SR & 0X40)==0);
//    USART1->DR = (uint8_t) ch;
//    return ch;
//}

#endif

//PA9--USART1_ TX	PA10--USART1_ RX	
void Serial_Init(void)
{
	//开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	//GPIO复用
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	//配置GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//配置USART
	USART_InitTypeDef USART_InitStucture;
	USART_InitStucture.USART_BaudRate = 115200;				//9600波特率
	USART_InitStucture.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
	USART_InitStucture.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStucture.USART_Parity = USART_Parity_No;		//无检验位
	USART_InitStucture.USART_StopBits = USART_StopBits_1;		//一位停止位
	USART_InitStucture.USART_WordLength = USART_WordLength_8b;  //8位数据位
	USART_Init(USART1,&USART_InitStucture);
	
	// 中断优先级配置
	NVIC_InitTypeDef NVIC_InitStucture;
	NVIC_InitStucture.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStucture.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStucture.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStucture.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStucture);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART1,ENABLE);
	
}

// 串口发送一个字节
void Serial_SendByte(uint16_t Byte)
{
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,Byte);
}

// 串口发送数组
void UART1_Send_Array(unsigned char send_array[], unsigned char num){
	unsigned char i=0;
	while(i!=num){
		USART_SendData(USART1, send_array[i]);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC)!= SET);
		i++;
	}
}


void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for(i=0; i<Length; i++)
	{
		Serial_SendByte(Array[i]);
	}
}

// 串口发送字符串
void Serial_SendString(char *String)
{
	uint8_t i;
	for(i=0; String[i] != '\0'; i++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Power(uint32_t X, uint32_t Y)
{
	uint32_t result = 1;
	while(Y--)
	{
		result *= X;
	}
	return result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for(i=0; i<Length; i++)
	{
		Serial_SendByte(Number / Serial_Power(10,Length - i - 1 ) % 10 + '0');
	}
}

// printf重定向函数
void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

uint16_t Serial_GetRxData(void)
{
	return Serial_RxData;
}

uint8_t Serial_GetRxFlag(void)
{
	if(Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

// 中断处理
void USART1_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL();
	OSIntEnter();
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == SET)
	{
		Serial_RxData = USART_ReceiveData(USART1);
		Serial_RxFlag = 1;
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	OS_EXIT_CRITICAL();
	OSIntExit();
}

// 打印四元数，调试时使用
void Printf_Qua(float q0, float q1, float q2, float q3){
	printf("q0:%f ", q0);
	printf("q1:%f ", q1);
	printf("q2:%f ", q2);
	printf("q3:%f ", q3);
}

// 调试姿态数据
void Send_Senser(int16_t ACC_X, int16_t ACC_Y, int16_t ACC_Z, int16_t GYRO_X,
					int16_t GYRO_Y, int16_t GYRO_Z, int16_t MAG_X, int16_t MAG_Y, int16_t MAG_Z)
{
	
	printf("ACC_X:%d ",ACC_X);
//	printf("ACC_Y:%d ",ACC_Y);
//	printf("ACC_Z:%d ",ACC_Z);
//	printf("GYRO_X:%d ",GYRO_X);
//	printf("GYRO_Y:%d ",GYRO_Y);
//	printf("GYRO_Z:%d ",GYRO_Z);
//	printf("MAG_X:%d	",MAG_X);
//	printf("MAG_Y:%d	",MAG_Y);
//	printf("MAG_Z:%d	",MAG_Z);
	
	
//	uint8_t i,_cnt=0;
	
//	sendBuf[_cnt++] = BYTE1(ACC_X);
//	sendBuf[_cnt++] = BYTE0(ACC_X);
//	sendBuf[_cnt++] = BYTE1(ACC_Y);
//	sendBuf[_cnt++] = BYTE0(ACC_Y);
//	sendBuf[_cnt++] = BYTE1(ACC_Z);
//	sendBuf[_cnt++] = BYTE0(ACC_Z);
//	
//	sendBuf[_cnt++] = BYTE1(GYRO_X);
//	sendBuf[_cnt++] = BYTE0(GYRO_X);
//	sendBuf[_cnt++] = BYTE1(GYRO_Y);
//	sendBuf[_cnt++] = BYTE0(GYRO_Y);
//	sendBuf[_cnt++] = BYTE1(GYRO_Z);
//	sendBuf[_cnt++] = BYTE0(GYRO_Z);
//	
//	sendBuf[_cnt++] = BYTE1(MAG_X);
//	sendBuf[_cnt++] = BYTE0(MAG_X);
//	sendBuf[_cnt++] = BYTE1(MAG_Y);
//	sendBuf[_cnt++] = BYTE0(MAG_Y);
//	sendBuf[_cnt++] = BYTE1(MAG_Z);
//	sendBuf[_cnt++] = BYTE0(MAG_Z);
//	
//	
//	for(i=0;i<_cnt;i++){
//		Serial_SendByte(sendBuf[i]);
//	}
}

/**
	按匿名数据帧协议向上位机发送四元数数据帧
**/
void Send_QuaBUFF(void)
{
	
	uint8_t i;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
//	uint8_t _cnt=0;							
	uint8_t BUFF[15];
	
	BUFF[0] = 0xAA;
	BUFF[1] = 0xFF;
	BUFF[2] = 0x04;
	BUFF[3] = 9;
	int16_t t = q0 * 10000;
	*((int16_t*)&BUFF[4]) = t;
	t = q1 * 10000;
	*((int16_t*)&BUFF[6]) = t;
	t = q2 * 10000;
	*((int16_t*)&BUFF[8]) = t;
	t = q3 * 10000;
	*((int16_t*)&BUFF[10]) = t;
	
	BUFF[12] = 0;
	
	for(i=0;i<BUFF[3]+4;i++) 
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[BUFF[3]+4] = sumcheck;	
	BUFF[BUFF[3]+5] = addcheck;
	
	
	UART1_Send_Array(BUFF, 15);
//	for(i=0;i<_cnt;i++)
//	{
//		Serial_SendByte(BUFF[i]);
//	}
}	

