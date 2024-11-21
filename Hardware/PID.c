#include "includes.h"

const float OUTER_INT_MAX = 40.0f;      // 外环积分限幅，对应角度误差累积和
const float INNER_INT_MAX = 40.0f;      // 内环积分限幅，对应角速度误差累积和
const float PWM_OUT_MAX = 300.0f;       // PWM 输出限幅，300.0 对应 30% 的油门
const float TAKEOFF_THROTTLE = 1450.0f; // 起飞油门，飞行时的油门应大于此值

// roll内环、roll外环、 pitch内环、 pitch外环、 yaw内环、 yaw外环
PID_t rollInner, rollOuter, pitchInner, pitchOuter, yawInner, yawOuter;
// 期望的roll，pitch，yaw
float expRoll, expPitch, expYaw, expMode;
// PID输出的油门控制量
float pidRoll, pidPitch, pidYaw, pidThr;

extern Angle angle;
extern float deltaT, Hight;
extern float motor1, motor2, motor3, motor4;
extern uint32_t PWM_IN_CH[5];

int correctFlag = -1;		// 机械校准

enum YAW_STATE {
    YAW_STABLE, // 油门大于 TAKEOFF_THROTTLE 且期望角速度为 0，此时采用串级 PID 控制
    YAW_ROTATE  // 否则为此状态，此时采用单环 PID 控制
};
enum FLY_STATE {
    HOVER,      // 悬停
    NOT_HOVER   // 非悬停
};
enum CH5_STATE { // 遥控器 CH5 三档开关状态，状态发生改变则进行机械校准
    CH5_DOWN,
    CH5_MIDDLE,
    CH5_UP
};

enum YAW_STATE yawState = YAW_STABLE;
enum FLY_STATE flyState = NOT_HOVER;
enum CH5_STATE ch5State = CH5_DOWN;
float targetYaw = 0.0f, targetHeight = 200.0f;

void PID_Init(void){
	// roll
    rollOuter.Kp = 5.0f;
    rollOuter.Ki = 2.0f;
    rollOuter.Kd = 0.3f;

    rollInner.Kp = 1.9f;
    rollInner.Ki = 0.0f;
    rollInner.Kd = 25.0f;

    // pitch
    pitchOuter.Kp = 5.0f;
    pitchOuter.Ki = 2.0f;
    pitchOuter.Kd = 0.3f;

    pitchInner.Kp = 1.9f;
    pitchInner.Ki = 0.0f;
    pitchInner.Kd = 25.0f;

    // Yaw
    yawOuter.Kp = 5.0f;
    yawOuter.Ki = 2.0f;
    yawOuter.Kd = 0.3f;

    yawInner.Kp = 1.9f;
    yawInner.Ki = 0.0f;
    yawInner.Kd = 25.0f;
}

static float limit(float x, float minv, float maxv) {
	return x < minv ? minv : (x > maxv ? maxv : x);
}

float serialPIDcontrol(float errTheta, float omega, PID_t *outer, PID_t *inner) {
	outer->err = errTheta;
	if (expMode > TAKEOFF_THROTTLE) {
		outer->errSum = limit(outer->errSum + outer->err * deltaT, -OUTER_INT_MAX, OUTER_INT_MAX);
	} else {
		outer->errSum = 0.0f;
	}
	outer->outputP = outer->Kp * outer->err;
	outer->outputI = outer->Ki * outer->errSum;
	outer->outputD = outer->Kd * omega;
	outer->output = outer->outputP + outer->outputI + outer->outputD;
	
	inner->err = outer->output - omega;
	if (expMode > TAKEOFF_THROTTLE) {
        inner->errSum = limit(inner->errSum + inner->err * deltaT, -INNER_INT_MAX, INNER_INT_MAX);
    } else {
        inner->errSum = 0.0f;
    }
	
	inner->outputP = inner->Kp * inner->err;
	inner->outputI = inner->Ki * inner->errSum;
	inner->outputD = inner->Kd * (inner->err - inner->errLast);
	
	inner->output = inner->outputP + inner->outputI + inner->outputD;
	inner->errLast = inner->err;
	if (++inner->cnt == 10) {
		inner->errLast = inner->err;
		inner->cnt = 0;
	}
	inner->output = limit(inner->output, -PWM_OUT_MAX, PWM_OUT_MAX);
	
	return inner->output;
}

float singlePIDcontrol(float omega, PID_t *pid) {
	pid->err = expYaw - omega;
	if (expMode > TAKEOFF_THROTTLE) {
        pid->errSum = limit(pid->errSum + pid->err * deltaT, -OUTER_INT_MAX, OUTER_INT_MAX);
    } else {
        pid->errSum = 0.0f;
    }
	pid->outputP = pid->Kp * pid->err;
	pid->outputI = pid->Ki * pid->errSum;
	pid->outputD = pid->Kd * (pid->err - pid->errLast);
	if (++pid->cnt == 10) {
		pid->errLast = pid->err;
		pid->cnt = 0;
	}
	pid->output = pid->outputP + pid->outputI + pid->outputD;
	pid->output = limit(pid->output, -PWM_OUT_MAX, PWM_OUT_MAX);
	
	return pid->output;
}

void PIDControl(float *gyroFiltered) {
	uint32_t PWMInCh1, PWMInCh2, PWMInCh3, PWMInCh4;
	// 限幅
	PWMInCh1 = limit(PWM_IN_CH[0], 1000, 2000);
	PWMInCh2 = limit(PWM_IN_CH[1], 1000, 2000);
	PWMInCh3 = limit(PWM_IN_CH[2], 1000, 2000);
	PWMInCh4 = limit(PWM_IN_CH[3], 1000, 2000);
	
	expRoll = (float)((PWMInCh4 - 1500) * 0.03f);	// 最大20度
	expPitch = (float)((PWMInCh2 - 1500) * 0.04f);	// 最大20度
	
	expYaw = (float)((PWMInCh1 - 1500) * 0.02f);	// 最大10度每秒
	expMode = PWMInCh3; 		// 油门在 [1000, 2000]
	
	enum CH5_STATE curCh5State = PWM_IN_CH[4] < 1400 ? CH5_DOWN : (PWM_IN_CH[4] < 1600 ? CH5_MIDDLE : CH5_UP);
	if (curCh5State != ch5State) {
		ch5State = curCh5State;
		correctFlag = 0;
	}
	
	pidRoll = serialPIDcontrol(expRoll - angle.roll, gyroFiltered[0] * RAD_TO_DEGREE, &rollOuter, &rollInner);
	pidPitch = serialPIDcontrol(expPitch - angle.pitch, gyroFiltered[1] * RAD_TO_DEGREE, &pitchOuter, &pitchInner);
	
	if (expMode > TAKEOFF_THROTTLE && fabsf(expYaw) < 5.0f) {	// 飞行稳定
        if (yawState != YAW_STABLE) {
            yawState = YAW_STABLE;
            targetYaw = angle.yaw;
            yawOuter.errSum = yawInner.errSum = 0.0f;
        }
        float errYaw = targetYaw - angle.yaw;
        if (errYaw < -180.0f) {
            errYaw += 360.0f;
        } else if (errYaw > 180.0f) {
            errYaw -= 360.0f;
        }
        pidYaw = serialPIDcontrol(errYaw, gyroFiltered[2] * RAD_TO_DEGREE, &yawOuter, &yawInner);
    } else {
        if (yawState != YAW_ROTATE) {
            yawState = YAW_ROTATE;
            yawOuter.errSum = yawInner.errSum = 0.0f;
        } 
        pidYaw = singlePIDcontrol(gyroFiltered[2] * RAD_TO_DEGREE, &yawInner);
    }
	
	motor1 = limit(expMode + pidRoll - pidPitch - pidYaw + pidThr, 1000, 2000);
	motor2 = limit(expMode - pidRoll - pidPitch + pidYaw + pidThr, 1000, 2000);
    motor3 = limit(expMode - pidRoll + pidPitch - pidYaw + pidThr, 1000, 2000);
    motor4 = limit(expMode + pidRoll + pidPitch + pidYaw + pidThr, 1000, 2000);
	
	if (expMode <= 1050 || angle.pitch >= 65 || angle.pitch <= -65 || angle.roll >= 65 || angle.roll <= -65) {
		motor1 = 1000;
		motor2 = 1000;
		motor3 = 1000;
		motor4 = 1000;
	}
}