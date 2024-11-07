#include "stm32f4xx.h"                  // Device header

/*该c文件用于测量PWM信号的频率和占空比*/
//测试引脚为PB6

void ICa_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
    
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
	
	TIM_InternalClockConfig(TIM4);
	
	//TIM_DeInit(TIM4);
	
    
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);

	TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
	
	TIM_Cmd(TIM4, ENABLE);
	
}

uint32_t ICa_GetFreq(void)
{
	return 1000000 / (TIM_GetCapture1(TIM4) + 1);
}

uint32_t ICa_GetDuty(void)
{
	return (TIM_GetCapture2(TIM4) + 1) * 100 / (TIM_GetCapture1(TIM4) + 1);
}
