#include "includes.h"
//#include "os_trace_events.h"			//SystemView

volatile float deltaT = 0.003f;
int16_t Acel[3], Gyro[3], Mag[3];		// 原始加速度角速度磁场数据
float acc[3], gyro[3], mag[3];			// 据量程转成弧度及零偏校准后的数据
float gyroFiltered[3] = {0.0f, 0.0f, 0.0f};
Angle angle;
float Height;							// 
float Pressure;							// 温度补偿大气压
float Temperature; 						// 实际温度

uint32_t PWM_IN_CH[5];			//4个输入捕获通道高电平计数值
float motor1, motor2, motor3, motor4; 

uint8_t sendBuf[20];			//串口发送数据帧
OS_EVENT* IICMutex;


/*	任务优先级	*/
#define TASK_STARTUP_PRIO	20
#define MUTEX_IIC_PRIO		21
#define TASK_ANGEL_PRIO		22
#define TASK_MOTOR_PRIO		23
#define TASK_COM_PRIO		24
#define TASK_LED_PRIO		25

/*	任务栈大小	*/
#define TASK_STARTUP_STK_SIZE		128
#define TASK_ANGEL_STK_SIZE			128
#define TASK_MOTOR_STK_SIZE			128
#define TASK_COM_STK_SIZE			128
#define TASK_LED_STK_SIZE			64

/*	任务栈定义	*/
OS_STK	TASK_STARTUP_STK[TASK_STARTUP_STK_SIZE];
OS_STK	TASK_ANGEL_STK[TASK_ANGEL_STK_SIZE];
OS_STK	TASK_MOTOR_STK[TASK_MOTOR_STK_SIZE];
OS_STK  TASK_COM_STK[TASK_COM_STK_SIZE];
OS_STK	TASK_LED_STK[TASK_LED_STK_SIZE];

/*	任务	*/
void Task_Startup(void *pdata);
void Task_Angel(void *pdata);
void Task_Motor(void *pdata);
void Task_Com(void *pdata);
void Task_LED(void *pdata);



int main(void){
	
	OSInit();
//	OSTaskCreateExt(Task_Angel,(void *)0, &TASK_ANGEL_STK[TASK_ANGEL_STK_SIZE-1], TASK_ANGEL_PRIO, TASK_ANGEL_PRIO, TASK_ANGEL_STK,TASK_ANGEL_STK_SIZE,(void *)0,OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
//	OSTaskCreateExt(Task_Motor,(void *)0, &TASK_MOTOR_STK[TASK_MOTOR_STK_SIZE-1], TASK_MOTOR_PRIO, TASK_MOTOR_PRIO, TASK_MOTOR_STK, TASK_MOTOR_STK_SIZE, (void *)0,OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
//	OSTaskCreateExt(Task_Com,(void *)0, &TASK_COM_STK[TASK_COM_STK_SIZE-1], TASK_COM_PRIO, TASK_COM_PRIO, TASK_COM_STK, TASK_COM_STK_SIZE, (void *)0,OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
//	OSTaskCreateExt(Task_LED,(void *)0,&TASK_LED_STK[TASK_LED_STK_SIZE-1],TASK_LED_PRIO, TASK_LED_PRIO, TASK_LED_STK, TASK_LED_STK_SIZE, (void *)0,OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
//	
//	INT8U err;
//	OSTaskNameSet(TASK_ANGEL_PRIO, (INT8U *)"Task_Angel", &err);
//	OSTaskNameSet(TASK_MOTOR_PRIO, (INT8U *)"TASK_MOTOR", &err);
//	OSTaskNameSet(TASK_COM_PRIO, (INT8U *)"TASK_COM", &err);
//	OSTaskNameSet(TASK_LED_PRIO, (INT8U *)"TASK_LED", &err);
	
	OSTaskCreate(Task_Startup,(void *)0, &TASK_STARTUP_STK[TASK_STARTUP_STK_SIZE-1], TASK_STARTUP_PRIO);
	OSStart();
	
	return 0;
}


void Task_Startup(void *pdata){
	
	INT8U err;
	
	Delay_Init(84);			// Delay函数初始化
	SysTick_Init();			// 滴答定时器初始化
//	SysTick1_Init(84);
	
	Serial_Init();			// 串口初始化
	IIC_Init();				// IIC初始化
	MPU6050_Init();			
//	while(MPU6050_Init()!=1);
	
	HMC5883L_Init();
//	MS5611_Init();
//	TIM2_Init();
	
	PWM_Init();
	IC_Init();
	LED_Init();
//	Motor_Start();			// 启动电机
	
//	OS_TRACE_INIT();
//	OS_TRACE_START(); 
	
	OSTimeDlyHMSM(0,0,3,0);
	Open_Calib();
	
	IICMutex = OSMutexCreate(MUTEX_IIC_PRIO, &err);
	OSTaskCreateExt(Task_Angel,(void *)0, &TASK_ANGEL_STK[TASK_ANGEL_STK_SIZE-1], TASK_ANGEL_PRIO, TASK_ANGEL_PRIO, TASK_ANGEL_STK,TASK_ANGEL_STK_SIZE,(void *)0,OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(Task_Motor,(void *)0, &TASK_MOTOR_STK[TASK_MOTOR_STK_SIZE-1], TASK_MOTOR_PRIO, TASK_MOTOR_PRIO, TASK_MOTOR_STK, TASK_MOTOR_STK_SIZE, (void *)0,OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(Task_Com,(void *)0, &TASK_COM_STK[TASK_COM_STK_SIZE-1], TASK_COM_PRIO, TASK_COM_PRIO, TASK_COM_STK, TASK_COM_STK_SIZE, (void *)0,OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(Task_LED,(void *)0,&TASK_LED_STK[TASK_LED_STK_SIZE-1],TASK_LED_PRIO, TASK_LED_PRIO, TASK_LED_STK, TASK_LED_STK_SIZE, (void *)0,OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	
	
//	OSTaskNameSet(TASK_ANGEL_PRIO, (INT8U *)"Task_Angel", &err);
//	OSTaskNameSet(TASK_MOTOR_PRIO, (INT8U *)"TASK_MOTOR", &err);
//	OSTaskNameSet(TASK_COM_PRIO, (INT8U *)"TASK_COM", &err);
//	OSTaskNameSet(TASK_LED_PRIO, (INT8U *)"TASK_LED", &err);
	
	OSTaskDel(OS_PRIO_SELF);
}




void Task_Angel(void *pdata){
//	TIM2_Init();
//	TIM2->CNT = 0;
	while(1){
		
		MPU6050_GetData();
//		acc[0] = (float)((Acel[0] / 16384.0f) - accCali.Ox) * accCali.Sx;
//		acc[1] = (float)((Acel[1] / 16384.0f) - accCali.Oy) * accCali.Sy;
//		acc[2] = (float)((Acel[2] / 16384.0f) - accCali.Oz) * accCali.Sz;
		acc[0] = (float)(Acel[0] - accCali.Ox) / 16384.0f;
		acc[1] = (float)(Acel[1] - accCali.Oy) / 16384.0f;
		acc[2] = (float)(Acel[2] + (16384.0f - accCali.Oz)) / 16384.0f;
//		acc[0] = (float)(Acel[0] - accCali.Ox);
//		acc[1] = (float)(Acel[1] - accCali.Oy);
//		acc[2] = (float)(Acel[2] - accCali.Oz);
//		acc[0] = (float)(Acel[0]);
//		acc[1] = (float)(Acel[1]);
//		acc[2] = (float)(Acel[2]);
		
		gyro[0] = ((float)Gyro[0]) * RAW_TO_RAD - gyroCali.Ox;
		gyro[1] = ((float)Gyro[1]) * RAW_TO_RAD - gyroCali.Oy;
		gyro[2] = ((float)Gyro[2]) * RAW_TO_RAD - gyroCali.Oz;
//		gyro[0] = ((float)Gyro[0] - gyroCali.Ox) * RAW_TO_RAD;
//		gyro[1] = ((float)Gyro[1] - gyroCali.Oy) * RAW_TO_RAD;
//		gyro[2] = ((float)Gyro[2] - gyroCali.Oz) * RAW_TO_RAD;

		memcpy(gyroFiltered, gyro, sizeof(gyro));
		
		HMC5883L_GetData();
//		mag[0] = (float)((Mag[0] / 1090.0f) - magCali.Ox) * magCali.Sx;
//		mag[1] = (float)((Mag[1] / 1090.0f) - magCali.Oy) * magCali.Sy;
//		mag[2] = (float)((Mag[2] / 1090.0f) - magCali.Oz) * magCali.Sz;
		mag[0] = (float)(Mag[0] / 1090.0f);
		mag[1] = (float)(Mag[1] / 1090.0f);
		mag[2] = (float)(Mag[2] / 1090.0f);
//		mag[0] = (float)(Mag[0]);
//		mag[1] = (float)(Mag[1]);
//		mag[2] = (float)(Mag[2]);

//		deltaT = TIM2->CNT / 1000000.0f;
////		printf("deltaT: %d\r\n", TIM2->CNT);
//		TIM2->CNT = 0;
		
//		MS5611_GetData();
//		
		PrepareData();
		
		if (!Calib_Status()){
			MadgwickAHRSupdate(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]);
//			Height_Update(Pressure);
			Send_QuaBUFF();
		}
//		MadgwickAHRSupdate(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]);
//		Send_QuaBUFF();
//		MadgwickAHRSupdateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);
//		Send_Senser(Acel[0], Acel[1], Acel[2], gyro[0], gyro[1], gyro[2], Mag[0], Mag[1], Mag[2]);
		OSTimeDly(10);
	}
}

void Task_Motor(void *pdata){
	while(1){
//		PWM_OUT();
		OSTimeDly(3);
	}
}

void Task_Com(void *pdata){
	while(1){
//		Send_Senser(Acel[0], Acel[1], Acel[2], gyro[0], gyro[1], gyro[2], Mag[0], Mag[1], Mag[2]);
//		printf("%f",q0);
//		Printf_Qua(q0, q1, q2, q3);
//		Send_QuaBUFF();
		OSTimeDly(10);
	}
}


void Task_LED(void *pdata){
	while(1){	
		LED_ON();
		OSTimeDlyHMSM(0, 0, 1, 0);
		LED_OFF();
		OSTimeDlyHMSM(0, 0, 1, 0);
	}
}


