#include "includes.h"
//#include "os_trace_events.h"			//SystemView


MPU6050_DataTypeDef Acc_Gyro;	//原始加速度角速度数据
//HMC5883L_DataTypeDef Mag;		//原始磁力数据
Float_t fGyro;					//角速度数据（弧度）
int16_t Acel[3], Gyro[3], Mag[3];
float acc[3], gyro[3], mag[3];


uint32_t PWM_IN_CH[4];			//4个输入捕获通道高电平计数值

uint8_t sendBuf[20];			//串口发送数据帧

// 初始化四元数
//extern volatile double q[4];



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

OS_EVENT* IICMutex;
//OS_EVENT *sem;

//	sem = OSSemCreate(1);
//uint8_t i;
//int8_t buff[4];


int main(void){
	
//	Delay_Init(84);			//Delay初始化
//	SysTick_Init();			//滴答定时器初始化
//	Serial_Init();			//串口初始化
//	IIC_Init();				
//	MPU6050_Init();
//	HMC5883L_Init();
//	PWM_Init();
//	IC_Init();
//	LED_Init();
//	
//	Motor_Start();
//	
//	OS_TRACE_INIT();
//	OS_TRACE_START(); 
//	
//	OSTimeDlyHMSM(0,0,5,0);
	
	
	
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
	

	Delay_Init(84);			//Delay初始化
	SysTick_Init();			//滴答定时器初始化
//	SysTick1_Init(84);
	
	Serial_Init();			//串口初始化
	IIC_Init();
	MPU6050_Init();
//	while(MPU6050_Init()!=1);
	
	HMC5883L_Init();
	PWM_Init();
	IC_Init();
	LED_Init();
	
	
	Motor_Start();
	
	OS_TRACE_INIT();
	OS_TRACE_START(); 
	
	OSTimeDlyHMSM(0,0,5,0);
	
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
	while(1){
		MPU6050_GetData();
		acc[0] = (float)(Acel[0]*ACC_RANGE/65536.0f);
		acc[1] = (float)(Acel[1]*ACC_RANGE/65536.0f);
		acc[2] = (float)(Acel[2]*ACC_RANGE/65536.0f);
//		acc[0] = (float)(Acel[0]);
//		acc[1] = (float)(Acel[1]);
//		acc[2] = (float)(Acel[2]);
		
		gyro[0] = ((float)Gyro[0]) * RAW_TO_RAD;
		gyro[1] = ((float)Gyro[1]) * RAW_TO_RAD;
		gyro[2] = ((float)Gyro[2]) * RAW_TO_RAD;
		
		HMC5883L_GetData();
		mag[0] = (float)(Mag[0] / 1090.0f);
		mag[1] = (float)(Mag[1] / 1090.0f);
		mag[2] = (float)(Mag[2] / 1090.0f);
//		mag[0] = (float)(Mag[0]);
//		mag[1] = (float)(Mag[1]);
//		mag[2] = (float)(Mag[2]);
		
		MadgwickAHRSupdate(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag[1], -mag[0], mag[2]);
//		MadgwickAHRSupdateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);
//		printf("q0: 			",q0);
//		printf("ACC_X:%d ",Acel[0]);
//		printf("ACC_Y:%d ",Acel[1]);
//		printf("ACC_Z:%d ",Acel[2]);
//		Send_Senser(Acel[0], Acel[1], Acel[2], gyro[0], gyro[1], gyro[2], Mag[0], Mag[1], Mag[2]);
		Send_SiYSBUFF();
		OSTimeDly(5);
	}
}

void Task_Motor(void *pdata){
	while(1){
		PWM_OUT();
		OSTimeDly(3);
	}
}


void Task_Com(void *pdata){
	while(1){
//		Send_Senser(Acel[0], Acel[1], Acel[2], gyro[0], gyro[1], gyro[2], Mag[0], Mag[1], Mag[2]);
//		printf("%f",q0);
//		Send_si(q0, q1, q2, q3);
//		Send_SiYSBUFF();
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

