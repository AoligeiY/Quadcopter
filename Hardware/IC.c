#include "stm32f4xx.h"                  // Device header
#include "ucos_ii.h"
#include "os_cpu.h"

extern uint32_t PWM_IN_CH[4];

void IC_Init(void)
{
    /**开启外设时钟**/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
    /**配置GPIOB**/
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    /**复用GPIOB为TIM4**/
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM4);
	
    /**初始化TIM4，开启定时器内部时钟**/
	TIM_DeInit(TIM4);
	
    /**TIM4时基单元配置**/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 5000 - 1;		//ARR设为5000-1
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;		//记一次数需要(1/1M)s
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	//TIM4->CH1	PB6
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0B;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	//TIM4->CH2	PB7
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0x0B;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	//TIM4->CH3	PB8
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICFilter = 0x0B;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	//TIM4->CH4	PB9
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICFilter = 0x0B;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM4, TIM_IT_CC1 | TIM_IT_CC2| TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_Update);
	
    /**配置NVIC**/
	NVIC_InitTypeDef NVIC_InitTypeDefStructure;
	NVIC_InitTypeDefStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitTypeDefStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitTypeDefStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitTypeDefStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitTypeDefStructure);
	
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	TIM_ARRPreloadConfig(TIM4,DISABLE);
	
    /**中断使能**/
	TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 |TIM_IT_Update , ENABLE);

    //使能定时器
	TIM_Cmd(TIM4,ENABLE);
	
}


/* 寄存器版本 */
//void IC_Init(void){
//	/**初始化GPIO*/
//	/*
//	测试PA0，PA1，PB8，PB9
//	PA0 --->TIM2_CH1
//	PA1 --->TIM2_CH2
//	PB6 --->TIM4_CH1
//	PB7 --->TIM4_CH2
//	PB8 --->TIM4_CH3
//	PB9 --->TIM4_CH4
//	
//	*/
////	RCC->AHB1ENR |= (1<<0);//打开GPIOA时钟
//	RCC->AHB1ENR |= (1<<1);//打开GPIOB时钟
////	RCC->APB1ENR |= (1<<0);//打开TIM2时钟
//	RCC->APB1ENR |= (1<<2);//打开TIM4时钟
//	
////	GPIOA->MODER &= ~(0x0f);//将PA0、1MOIDER位清零
////	GPIOA->MODER |= (0x0a);//将PA0、1设置位复用模式
////	GPIOA->OSPEEDR |= (0x0a);//设置四个PA口为高速  
////	GPIOA->PUPDR |= (0x0a);//GPIO四个引脚设置为下拉，因为要判断高电平的持续时间
//	
//	GPIOB->MODER &= ~(0xff<<12);//将PB6,PB7,PB8，PB9MODER位清零
//	GPIOB->MODER |= (0xaa<<12);//将PB6,PB7,PB8，PB9设置为复用模式
//	GPIOB->OSPEEDR |= (0xaa<<12);//将PB6,PB7,PB8,9为高速
//	GPIOB->PUPDR |= (0xaa<<12);//设置为下拉，因为要判断高电平的持续时间
//	
////	GPIOA->AFR[0] &= ~(0xff);//将PA01复用位清零
////	GPIOA->AFR[0] |= (0x1<<0);//PA0复用为TIM2通道1
////	GPIOA->AFR[0] |= (0x1<<4);//PA1复用为TIM2通道2

//	
//	GPIOB->AFR[1] &= ~(0xff);//将PB复用位清零
//	GPIOB->AFR[0] &= ~(0xffU<<24);
//	GPIOB->AFR[0] |= (0x2<<24);//PB6复用为TIM4通道1
//	GPIOB->AFR[0] |= (0x2<<28);//PB7复用为TIM4通道2
//	GPIOB->AFR[1] |= (0x1<<1);//PB8复用为TIM4通道3
//	GPIOB->AFR[1] |= (0x1<<5);//PB9复用为TIM4通道4
//	
//	
//	/**TIM2时钟配置*/
////	TIM2->CR1 &= ~(0Xf<<1);//UEVenable，更新请求源，关闭单脉冲，向上计数
////	TIM2->CR1 |= (0x1<<7);//预装载影子寄存器
////	TIM2->SMCR &= ~(0x7);//选择内部时钟 Slave mode disable,这样定时器将不会受到触发事件的影响
////	
////	TIM2->PSC = psc;//设置分频系数
////	TIM2->ARR = arr;//设置重装载寄存器
////	
////	
////	
////	/**通道一*/
////	TIM2->CCMR1 &= ~(0x3<<0);//先清零
////	TIM2->CCMR1 |= (0x01);//CC1S设置为输入模式，将CC1映射到TI1
////	TIM2->CCMR1 &= ~(0x3<<2);//设置IC1PSC为不分频，只要每检测到一个边沿就触发一次捕获
////	TIM2->CCMR1 &= ~(0xf<<4);//设置通道滤波为0000 不做滤波处理，只要采集到边沿就触发
////	
////	TIM2->CCER |= (0x1<<0);//捕获使能CC1E
////	TIM2->CCER &= ~(0x1<<1);//捕获上升沿CC1P
////	
////	/*通道二*/
////	TIM2->CCMR1 &= ~(0x3<<8);
////	TIM2->CCMR1 |= (0x01<<8);//CC2S将IC2映射到TI2
////	TIM2->CCMR1 &= ~(0x3<<10);//IC1PSC不分频
////	TIM2->CCMR1 &= ~(0xf<<12);//IC1F不做滤波处理
////	
////	TIM2->CCER |= (0x1<<4);//CC2E
////	TIM2->CCER &= ~(0x1<<5);//CC2P
////	
////	TIM2->DIER |= (0x1<<0);//使能更新中断更新__HAL_TIM_ENABLE_IT
////	TIM2->DIER |= (0x1<<1);//使能channel1捕获中断HAL_TIM_IC_Start_IT
////	TIM2->DIER |= (0x1<<2);//使能channel2捕获中断

////	
////	TIM2->CR1 |= (0x1<<0);//使能CEN计数器
////	my_nvic_set_priority(TIM2_IRQn,1,3);
////	my_nvic_enable(TIM2_IRQn);
////	HAL_NVIC_SetPriority(TIM2_IRQn,1,3);//设置中断优先级
////	HAL_NVIC_EnableIRQ(TIM2_IRQn);//使能中断

////	 TIM2->CNT -= 0xffff; 
//	
//	
//	/**TIM4时钟配置*/
//	TIM4->CR1 &=~(0Xf<<1);//UEVenable，更新请求源，关闭单脉冲，向上计数
//	TIM4->CR1 |= (0x1<<7);//预装载影子寄存器
//	TIM4->SMCR &= ~(0x7);//选择内部时钟 Slave mode disable,这样定时器将不会受到触发事件的影响
//	
//	TIM4->PSC = 84-1;//设置分频系数
//	TIM4->ARR = 5000-1;//设置重装载
//	/**通道一*/
//	TIM4->CCMR1 &= ~(0x3<<0);//先清零
//	TIM4->CCMR1 |= (0x01);//CC1S设置为输入模式，将CC1映射到TI1
//	TIM4->CCMR1 &= ~(0x3<<2);//设置IC1PSC为不分频，只要每检测到一个边沿就触发一次捕获
//	TIM4->CCMR1 &= ~(0xf<<4);//设置通道滤波为0000 不做滤波处理，只要采集到边沿就触发
//	
//	TIM4->CCER |= (0x1<<0);//捕获使能CC1E
//	TIM4->CCER &= ~(0x1<<1);//捕获上升沿CC1P

//	/**通道二*/
//	TIM4->CCMR1 &= ~(0x3<<8);
//	TIM4->CCMR1 |= (0x01<<8);//CC2S将IC2映射到TI2
//	TIM4->CCMR1 &= ~(0x3<<10);//IC1PSC不分频
//	TIM4->CCMR1 &= ~(0xf<<12);//IC1F不做滤波处理
//	
//	TIM4->CCER |= (0x1<<4);//CC2E
//	TIM4->CCER &= ~(0x1<<5);//CC2P

//	/**通道三*/
//	TIM4->CCMR2 &= ~(0x3<<0);
//	TIM4->CCMR2 |= (0x01<<0);//将IC3映射到TI3
//	TIM4->CCMR2 &= ~(0x3<<2);//不分频
//	TIM4->CCMR2 &= ~(0xf<<4);//不滤波
//	
//	TIM4->CCER |= (0x1<<8);//捕获使能
//	TIM4->CCER &= ~(0x1<<9);//捕获上升沿
//	/**通道四*/
//	TIM4->CCMR2 &= ~(0x3<<8);
//	TIM4->CCMR2 |= (0x01<<8);//将IC3映射到TI3
//	TIM4->CCMR2 &= ~(0x3<<10);//不分频
//	TIM4->CCMR2 &= ~(0xf<12);
//	
//	TIM4->CCER |=(0x1<<12);
//	TIM4->CCER &= ~(0x1<<13);
//	
//	
//	TIM4->DIER |= (0x1<<0);//使能更新中断更新__HAL_TIM_ENABLE_IT
//	TIM4->DIER |= (0x1<<1);
//	TIM4->DIER |= (0x1<<2);
//	TIM4->DIER |= (0x1<<3);//使能channel3捕获中断
//	TIM4->DIER |= (0x1<<4);//使能channel4捕获中断
//	
//	TIM4->CR1 |= (0x1<<0);//使能CEN计数器
//	
//	NVIC_InitTypeDef NVIC_InitTypeDefStructure;
//	NVIC_InitTypeDefStructure.NVIC_IRQChannel = TIM4_IRQn;
//	NVIC_InitTypeDefStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitTypeDefStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitTypeDefStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_InitTypeDefStructure);
//	TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 |TIM_IT_Update , ENABLE);
//	
////	my_nvic_set_priority(TIM4_IRQn,1,3);
////	my_nvic_enable(TIM4_IRQn);
////	HAL_NVIC_SetPriority(TIM4_IRQn,1,3);//设置中断优先级
////	HAL_NVIC_EnableIRQ(TIM4_IRQn);//使能中断v
//	//TIM4->CNT = 0;

//}


uint8_t TIM4_CAPTURE_STA[4] = {0};		//初始值为0。0：此前为低电平,说明此时捕获到上升沿; 1: 此前为高电平，说明此时捕获下降沿
uint16_t TIM4_CAPTURE_OVF[4] = {0};		//CHx高电平期间，计数器溢出次数
uint16_t TIM4_CAPTURE_VAL[4][2] = {0};	//CHx捕获到上升沿与下降沿的CCRx的值

void TIM4_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL();
	OSIntEnter();
	//更新中断处理
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);		//清除更新中断标志位
		uint8_t i;
		for(i = 0; i < 4; i++)
		{
			if(TIM4_CAPTURE_STA[i] == 1)				//判断CHx已经处在高电平
			{
				if((TIM4_CAPTURE_OVF[i] & 0x3F) == 0x3F)	//高电平时间过长
				{	
					TIM4_CAPTURE_VAL[i][0] = 0;				//捕获时间舍弃
					TIM4_CAPTURE_OVF[i]=0;
				}else
				{
					TIM4_CAPTURE_OVF[i]++;			//溢出次数加一
				}			
			}	
		}
	}
	
	//CH1捕获
	if(TIM_GetFlagStatus(TIM4, TIM_IT_CC1) != RESET )			//通道一发生捕获
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC1);					//清除捕获中断标志位
		if(TIM4_CAPTURE_STA[0] == 0)								//捕获前为低电平,捕获上升沿
		{
			TIM4_CAPTURE_VAL[0][0] = TIM_GetCapture1(TIM4);		//存取上升沿的CNT值
			TIM4_CAPTURE_STA[0] = 1 ;							//更改捕获状态
			TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Falling);	//通道下降沿捕获
		}else{
			TIM4_CAPTURE_VAL[0][1] = TIM_GetCapture1(TIM4);		//存取下降沿的CNT值
			if(TIM4_CAPTURE_VAL[0][1]>TIM4_CAPTURE_VAL[0][0]){
				PWM_IN_CH[0] = TIM4_CAPTURE_VAL[0][1] - TIM4_CAPTURE_VAL[0][0] + TIM4_CAPTURE_OVF[0]*4999;
			}else{
				PWM_IN_CH[0] = TIM4_CAPTURE_OVF[0]*4999 - TIM4_CAPTURE_VAL[0][0] + TIM4_CAPTURE_VAL[0][1];
			}
			TIM4_CAPTURE_OVF[0] = 0 ;							//溢出次数清零
			TIM4_CAPTURE_STA[0] = 0 ;							//更改捕获状态
			TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Rising);	//通道上升沿沿捕获
		}
	}
	
	//CH2捕获
	if(TIM_GetFlagStatus(TIM4, TIM_IT_CC2) != RESET )		
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC2);					
		if(TIM4_CAPTURE_STA[1] == 0)			
		{
			TIM4_CAPTURE_VAL[1][0] = TIM_GetCapture2(TIM4);		
			TIM4_CAPTURE_STA[1] = 1 ;							
			TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Falling);	
		}else{
			TIM4_CAPTURE_VAL[1][1] = TIM_GetCapture2(TIM4);		
			if(TIM4_CAPTURE_VAL[1][1]>TIM4_CAPTURE_VAL[1][0]){
				PWM_IN_CH[1] = TIM4_CAPTURE_VAL[1][1] - TIM4_CAPTURE_VAL[1][0] + TIM4_CAPTURE_OVF[1]*4999;
			}else{
				PWM_IN_CH[1] = TIM4_CAPTURE_OVF[1]*4999 - TIM4_CAPTURE_VAL[1][0] + TIM4_CAPTURE_VAL[1][1];
			}
			TIM4_CAPTURE_OVF[1] = 0 ;							
			TIM4_CAPTURE_STA[1] = 0 ;					
			TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Rising);	
		}
	}
	
	
	//CH3捕获
	if(TIM_GetFlagStatus(TIM4, TIM_IT_CC3) != RESET )		
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC3);					
		if(TIM4_CAPTURE_STA[2] == 0)			
		{
			TIM4_CAPTURE_VAL[2][0] = TIM_GetCapture3(TIM4);		
			TIM4_CAPTURE_STA[2] = 1 ;							
			TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Falling);	
		}else{
			TIM4_CAPTURE_VAL[2][1] = TIM_GetCapture3(TIM4);		
			if(TIM4_CAPTURE_VAL[2][1]>TIM4_CAPTURE_VAL[2][0]){
				PWM_IN_CH[2] = TIM4_CAPTURE_VAL[2][1] - TIM4_CAPTURE_VAL[2][0] + TIM4_CAPTURE_OVF[2]*4999;
			}else{
				PWM_IN_CH[2] = TIM4_CAPTURE_OVF[2]*4999 - TIM4_CAPTURE_VAL[2][0] + TIM4_CAPTURE_VAL[2][1];
			}
			TIM4_CAPTURE_OVF[2] = 0 ;							
			TIM4_CAPTURE_STA[2] = 0 ;					
			TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Rising);	
		}
	}
	
	
	//CH3捕获
	if(TIM_GetFlagStatus(TIM4, TIM_IT_CC4) != RESET )		
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC4);					
		if(TIM4_CAPTURE_STA[3] == 0)			
		{
			TIM4_CAPTURE_VAL[3][0] = TIM_GetCapture4(TIM4);		
			TIM4_CAPTURE_STA[3] = 1 ;							
			TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Falling);	
		}else{
			TIM4_CAPTURE_VAL[3][1] = TIM_GetCapture4(TIM4);		
			if(TIM4_CAPTURE_VAL[3][1]>TIM4_CAPTURE_VAL[3][0]){
				PWM_IN_CH[3] = TIM4_CAPTURE_VAL[3][1] - TIM4_CAPTURE_VAL[3][0] + TIM4_CAPTURE_OVF[3]*4999;
			}else{
				PWM_IN_CH[3] = TIM4_CAPTURE_OVF[3]*4999 - TIM4_CAPTURE_VAL[3][0] + TIM4_CAPTURE_VAL[3][1];
			}
			TIM4_CAPTURE_OVF[3] = 0 ;							
			TIM4_CAPTURE_STA[3] = 0 ;					
			TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Rising);	
		}
	}
	OS_EXIT_CRITICAL();
	OSIntExit();
}



/* 该部分代码功能与上实现基本一致，但容错率较上低 */

//void TIM4_IRQHandler(void)
//{

	//更新中断
//	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET){
//		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);		//清除更新中断标志位
//		uint8_t i;
//		for(i = 0; i < 4; i++)
//		{
//			if(TIM4_CAPTURE_STA[i] == 1)				//判断CHx已经处在高电平
//			{
//					TIM4_CAPTURE_OVF[i]++;							//溢出次数加一
//			}	
//		}
//	}

//	CH1捕获
//	if(TIM_GetFlagStatus(TIM4, TIM_IT_CC1) != RESET )			//通道一发生捕获
//	{
//		TIM_ClearITPendingBit(TIM4,TIM_IT_CC1);					//清除捕获中断标志位
//		if(TIM4_CAPTURE_STA[0] == 0)								//捕获前为低电平,捕获上升沿
//		{
//			TIM4_CAPTURE_VAL[0][0] = TIM_GetCapture1(TIM4);		//存取上升沿的CNT值
//			TIM4_CAPTURE_STA[0] = 1 ;							//更改捕获状态
//			TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Falling);	//通道下降沿捕获
//		}else{
//			TIM4_CAPTURE_VAL[0][1] = TIM_GetCapture1(TIM4);		//存取下降沿的CNT值
//			PWM_IN_CH[0] = TIM4_CAPTURE_VAL[0][1] - TIM4_CAPTURE_VAL[0][0] + TIM4_CAPTURE_OVF[0] * 65535; 		//计算脉冲宽度
//			TIM4_CAPTURE_OVF[0] = 0 ;							//溢出次数清零
//			TIM4_CAPTURE_STA[0] = 0 ;							//更改捕获状态
//			TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Rising);	//通道上升沿沿捕获
//		}
//	}
//	
//	//CH2捕获
//	if(TIM_GetFlagStatus(TIM4, TIM_IT_CC2) != RESET )
//	{
//		TIM_ClearITPendingBit(TIM4,TIM_IT_CC2);
//		if(TIM4_CAPTURE_STA[1] == 0)
//		{
//			TIM4_CAPTURE_VAL[1][0] = TIM_GetCapture2(TIM4);
//			TIM4_CAPTURE_STA[1] = 1 ;
//			TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Falling);
//		}else{
//			TIM4_CAPTURE_VAL[1][1] = TIM_GetCapture2(TIM4);
//			PWM_IN_CH[1] = TIM4_CAPTURE_VAL[1][1] - TIM4_CAPTURE_VAL[1][0] + TIM4_CAPTURE_OVF[1] * 65535; //计算脉冲宽度
//			TIM4_CAPTURE_OVF[1] = 0 ;
//			TIM4_CAPTURE_STA[1] = 0 ;
//			TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Rising);
//		}
//	}
//	
//	//CH3捕获
//	if(TIM_GetFlagStatus(TIM4, TIM_IT_CC3) != RESET )
//	{
//		TIM_ClearITPendingBit(TIM4,TIM_IT_CC3);
//		if(TIM4_CAPTURE_STA[2] == 0)
//		{
//			TIM4_CAPTURE_VAL[2][0] = TIM_GetCapture3(TIM4);
//			TIM4_CAPTURE_STA[2] = 1 ;
//			TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Falling);
//		}else{
//			TIM4_CAPTURE_VAL[2][1] = TIM_GetCapture3(TIM4);
//			PWM_IN_CH[2] = TIM4_CAPTURE_VAL[2][1] - TIM4_CAPTURE_VAL[2][0] + TIM4_CAPTURE_OVF[2] * 65535; //计算脉冲宽度
//			TIM4_CAPTURE_OVF[2] = 0 ;
//			TIM4_CAPTURE_STA[2] = 0 ;
//			TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Rising);
//		}
//	}
//	
//	//CH4捕获
//	if(TIM_GetFlagStatus(TIM4, TIM_IT_CC4) != RESET )
//	{
//		TIM_ClearITPendingBit(TIM4,TIM_IT_CC4);
//		if(TIM4_CAPTURE_STA[3] == 0)
//		{
//			TIM4_CAPTURE_VAL[3][0] = TIM_GetCapture4(TIM4);
//			TIM4_CAPTURE_STA[3] = 1 ;
//			TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Falling);
//		}else{
//			TIM4_CAPTURE_VAL[3][1] = TIM_GetCapture4(TIM4);
//			PWM_IN_CH[3] = TIM4_CAPTURE_VAL[3][1] - TIM4_CAPTURE_VAL[3][0] + TIM4_CAPTURE_OVF[3] * 65535; //计算脉冲宽度
//			TIM4_CAPTURE_OVF[3] = 0 ;
//			TIM4_CAPTURE_STA[3] = 0 ;
//			TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Rising);
//		}
//	}

//}
