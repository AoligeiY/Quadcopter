#include "stm32f4xx.h"                  // Device header

//PA5
//PC9
void LED_Init(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}
	
void LED_ON(void){
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
}

void LED_OFF(void){
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}

