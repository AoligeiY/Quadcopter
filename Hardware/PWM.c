#include "stm32f4xx.h"                  // Device header
#include "Delay.h"
#include "Serial.h"
#include "PWM.h"

//extern float motor1, motor2, motor3, motor4;

extern uint32_t PWM_IN_CH[4];

uint16_t M;		//串口传输数据




void PWM_Init(void)
{
	//开启时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	//配置GPIOA
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//配置GPIOB
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//引脚复用
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);	//CH1	横滚
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);	//CH2	俯仰
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3);	//CH3	油门
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3);	//CH4	偏航
	
	//置低电平
	GPIO_SetBits(GPIOA,GPIO_Pin_6 | GPIO_Pin_7);
	GPIO_SetBits(GPIOB,GPIO_Pin_0 | GPIO_Pin_1);
	
	//TIM3采用内部时钟
	TIM_InternalClockConfig(TIM3);
	
	//时基单元配置
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;	 		//ARR设为1000-1
	TIM_TimeBaseInitStructure.TIM_Prescaler = 540-1 ;			//接收机PWM信号频率为155Hz，故向电机输出155HzPWM	
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

	//配置输出比较
	TIM_OCInitTypeDef TIM_OCInitStructure;         
	TIM_OCStructInit(&TIM_OCInitStructure);						//赋初始值
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			//PWM1模式
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//使能
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);

	
	TIM_Cmd(TIM3,ENABLE);
	
	
}

/*  要想启动电机，首先得按一定占空比来输出给电机  */
void Motor_Start(void)
{
	TIM_SetCompare1(TIM3,100);		//占空比上界为10
	TIM_SetCompare2(TIM3,100);
	TIM_SetCompare3(TIM3,100);
	TIM_SetCompare4(TIM3,100);
	
	Delay_ms(20);
	
	TIM_SetCompare1(TIM3,50);		//占空比下届5
	TIM_SetCompare2(TIM3,50);
	TIM_SetCompare3(TIM3,50);
	TIM_SetCompare4(TIM3,50);
	
	Delay_ms(500);
	
	TIM_SetCompare1(TIM3,0);	
	TIM_SetCompare2(TIM3,0);
	TIM_SetCompare3(TIM3,0);
	TIM_SetCompare4(TIM3,0);
	
	Delay_ms(500);
}


//void PWM_SetDuty(uint16_t Duty)
//{
//	TIM3->CCR1 = Duty*(TIM3->ARR)/100;
//	TIM3->CCR2 = Duty*(TIM3->ARR)/100;
//	TIM3->CCR3 = Duty*(TIM3->ARR)/100;
//	TIM3->CCR4 = Duty*(TIM3->ARR)/100;
//}

// 串口数据设置占空比
//void PWM_SerialSetDuty(void)
//{
//	M = Serial_GetRxData();
//	if(M >= 5 && M <= 10)
//	{
//		TIM3->CCR1 = M*(TIM3->ARR)/100;
//		TIM3->CCR2 = M*(TIM3->ARR)/100;
//		TIM3->CCR3 = M*(TIM3->ARR)/100;
//		TIM3->CCR4 = M*(TIM3->ARR)/100;	
//	}
//}

//void PWM_OUT(void)
//{
//	TIM3->CCR1 = motor1*PWM_IN_OUT_VAL;
//	TIM3->CCR2 = motor2*PWM_IN_OUT_VAL;
//	TIM3->CCR3 = motor3*PWM_IN_OUT_VAL;
//	TIM3->CCR4 = motor4*PWM_IN_OUT_VAL;
//}

void PWM_OUT(void)
{
	TIM3->CCR1 = PWM_IN_CH[2]*PWM_IN_OUT_VAL;
	TIM3->CCR2 = PWM_IN_CH[2]*PWM_IN_OUT_VAL;
	TIM3->CCR3 = PWM_IN_CH[2]*PWM_IN_OUT_VAL;
	TIM3->CCR4 = PWM_IN_CH[2]*PWM_IN_OUT_VAL;
}

