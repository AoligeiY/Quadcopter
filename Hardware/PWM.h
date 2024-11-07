#ifndef _PWM_H_
#define _PWM_H_

//输入捕获计数器频率1MHz,那么高电平持续时间为 x/1M
//已知接收机PWM输出频率为155Hz，一个PWM周期为 1/155 s
//此时占空比为D，既有 D/155 = x/1M, x =6452D
//又因为PWM输出定时器（TIM3）ARR为1000，为了输出的占空比和输入的一样，则所需的CCRx值为 1000D
// x * PWM_IN_OUT_VAL = 1000D, 得PWM_IN_OUT_VAL = 0.155
//一句话总结PWM_IN_OUT_VAL是实现x转换成需要的CCR的参数

#define PWM_IN_OUT_VAL  0.155

void PWM_Init(void);
void Motor_Start(void);
void PWM_SetDuty(uint16_t Duty);
void PWM_SerialSetDuty(void);
void PWM_OUT(void);

//void PWM_SetCompare(uint16_t Compare);
//void PWM_SerPrescaler(uint16_t Prescalre);

#endif
