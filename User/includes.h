#ifndef __INCLUDES_H_
#define __INCLUDES_H_

//开启FPU,实际在core_cm4.h定义__FPU_USED
//#define		__CC_ARM			//1  		
//#define		__TARGET_FPU_VFP	//1    
//为了使用arm_math.h
#define		ARM_MATH_CM4			

#include "stm32f4xx.h"                  // Device header
#include <stdint.h>
#include "ucos_ii.h"
//#include "arm_math.h"
#include "core_cm4.h"
#include "os_trace.h"

#include "LED.h"
#include "IIC.h"
#include "Delay.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "Serial.h"
#include "PWM.h"
#include "IC.h"
#include "MadgwickAHRS.h"



#endif
