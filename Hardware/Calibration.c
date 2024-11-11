#include "includes.h"
#include <stdint.h>

#define M 300	//测试数据组数
#define N 6
const int n = N;
const int m = M;
extern int16_t Acel[3], Gyro[3], Mag[3];


//cali accCali = {0.011845, 0.973141, 0.019666, 0.996000, 0.636840, 0.915123};
cali accCali = {0, 1, 0, 1, 0, 1};
cali gyroCali = {-0.010501, 0.0f, -0.006461, 0.0f, 0.002650, 0.0f};
cali magCali = {0, 1, 0, 1, 0, 1};

static const float eps = 1e-6;
float LM_lamda = 0.1f;
float accData[M][3];
float gyroData[M][3];
float magData[M][3];


//float constrainFloat(float x, float low, float high) {
//    if (isnan(x)) {
//        return (low + high) * 0.5f;
//    }
//    return ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)));
//}


void PrepareData(void){
    printf("Prepare data....\r\n");
    OSTimeDlyHMSM(0,0,8,0);
    printf("Begin!....\r\n");
//    for(int i = 0; i < 300; i++){
//        MPU6050_GetData();
//        HMC5883L_GetData();
//        for (int j = 0; j < 3; j++){
//            accData[i][j] = Acel[j] / 16384.0f;
//            magData[i][j] = Mag[j] / 1090.0f;
//        }
//        printf("Finished: %d\r\n", i);
//        if (i != 300 -1)
//            OSTimeDlyHMSM(0,0,0,25);
//    }


//    OSTimeDlyHMSM(0,0,3,0);

    for(int i = 0; i < M; i++){
        MPU6050_GetData();
        for (int j = 0; j < 3; j++){
            gyroData[i][j] = Gyro[j] * RAW_TO_RAD;
        }
        printf("Finished: %d\r\n", i);
        if (i != 300 -1)
            OSTimeDlyHMSM(0,0,0,10);
    }

    float gyroSum[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < M; i++){
        for (int j = 0; j < 3; j++){
            gyroSum[j] += gyroData[i][j];
        }
    }
    gyroCali.Ox = gyroSum[0] / M;
    gyroCali.Oy = gyroSum[1] / M;
    gyroCali.Oz = gyroSum[2] / M;
    printf("Gyro offset: %f %f %f\r\n", gyroCali.Ox, gyroCali.Oy, gyroCali.Oz);
}


//void gaussElimination(float (*a)[N + 1]) {
//    for (int i = 0; i < n; ++i) {
//		int r = i;
//		for (int j = i + 1; j < n; ++j)
//		    if(fabsf(a[j][i]) > fabsf(a[r][i])) r = j;
//        if (fabsf(a[r][i]) < eps) {
//			printf("Error: gaussElimination no solution!\r\n");
//			return;
//		}
//		if(r != i) {
//            for (int j = 0; j < n + 1; ++j) {
//                float temp = a[i][j];
//                a[i][j] = a[r][j];
//                a[r][j] = temp;
//            }
//        }
//		for (int j = i + 1; j < n; ++j){
//			const float div = a[j][i] / a[i][i];
//            for (int k = i; k < n + 1; ++k)
//			    a[j][k] -= div * a[i][k];
//		}
//	}
//	for(int i = n - 1; i >= 0; --i){
//		for (int j = i + 1; j < n; ++j)
//		    a[i][n] -= a[j][n] * a[i][j];
//		a[i][n] /= a[i][i];
//	}
//} 

//// 高斯牛顿迭代
//void gaussNewton(cali *caliVal, float (*data)[3]) {
//    float Delta = 100.0f, DeltaNew = 0.0f; 
//    static float Jr[M][N], JrT[N][M]; // Jacobi 矩阵以及 Jacobi 矩阵的转置
//    static float r[M]; // 残差函数 r(beta)
//    static float delta[N][N + 1]; // 待求的迭代增量 delta，也是高斯消元的系数矩阵
//    float sum = 0.0f;

//    // 初始化    
//    static cali beta;
//    beta.Ox = beta.Oy = beta.Oz = 0.0f;
//    beta.Sx = beta.Sy = beta.Sz = 1.0f;
//    LM_lamda = 0.1f;

//    int cnt = 0;
//    // 迭代
//    while (Delta > eps && cnt < 100) {
//        // 计算 Jacobi 矩阵
//        for (int i = 0; i < m; ++i) {
//            Jr[i][0] = 2.0f * beta.Sx * beta.Sx * (beta.Ox - data[i][0]);
//            Jr[i][1] = 2.0f * (data[i][0] - beta.Ox) * (data[i][0] - beta.Ox) * beta.Sx;
//            Jr[i][2] = 2.0f * beta.Sy * beta.Sy * (beta.Oy - data[i][1]);
//            Jr[i][3] = 2.0f * (data[i][1] - beta.Oy) * (data[i][1] - beta.Oy) * beta.Sy;
//            Jr[i][4] = 2.0f * beta.Sz * beta.Sz * (beta.Oz - data[i][2]);
//            Jr[i][5] = 2.0f * (data[i][2] - beta.Oz) * (data[i][2] - beta.Oz) * beta.Sz;
//        }
//        // 计算 Jacobi 矩阵的转置
//        for (int i = 0; i < m; ++i) {
//            for (int j = 0; j < n; ++j) {
//                JrT[j][i] = Jr[i][j];
//            }
//        }
//        sum = 0.0;
//        // 计算残差函数 r(beta)
//        for (int i = 0; i < m; ++i) {
//            r[i] = beta.Sx * beta.Sx * (beta.Ox - data[i][0]) * (beta.Ox - data[i][0]) + 
//                   beta.Sy * beta.Sy * (beta.Oy - data[i][1]) * (beta.Oy - data[i][1]) + 
//                   beta.Sz * beta.Sz * (beta.Oz - data[i][2]) * (beta.Oz - data[i][2]) - 1.0f;
//            sum += r[i] * r[i];
//        }
//        // 计算 JrT * Jr 并作为系数矩阵放入 delta
//        for (int i = 0; i < n; ++i) {
//            for (int j = 0; j < n; ++j) {
//                delta[i][j] = 0;
//                for (int k = 0; k < m; ++k) {
//                    delta[i][j] += JrT[i][k] * Jr[k][j];
//                }
//            }
//        }
//        // 计算 JrT * r 并作为增广列放入 delta
//        for (int i = 0; i < n; ++i) {
//            delta[i][n] = 0;
//            for (int j = 0; j < m; ++j) {
//                delta[i][n] += JrT[i][j] * r[j];
//            }
//        }
//        // 加入 LM 因子
//        for (int i = 0; i < n; ++i) {
//            delta[i][i] += LM_lamda;
//        }
//        // 高斯消元
//        gaussElimination(delta);
//        // 计算 DeltaNew
//        DeltaNew = 0;
//        for (int i = 0; i < n; ++i) {
//            DeltaNew += delta[i][n] * delta[i][n];
//        }
//        if (DeltaNew < Delta) {
//            // LM 因子减小
//            LM_lamda /= 3.0;

//            // 更新 beta
//            beta.Ox -= delta[0][n];
//            beta.Sx -= delta[1][n];
//            beta.Oy -= delta[2][n];
//            beta.Sy -= delta[3][n];
//            beta.Oz -= delta[4][n];
//            beta.Sz -= delta[5][n];

//            Delta = DeltaNew;
//        } else {
//            // LM 因子增大
//            LM_lamda *= 3.0;
//            LM_lamda = constrainFloat(LM_lamda, 0, 1e10f);
//        }
//        
//        printf("After %d iterations: Ox = %f, Sx = %f, Oy = %f, Sy = %f, Oz = %f, Sz = %f lastSum = %f\r\n", 
//            ++cnt, beta.Ox, beta.Sx, beta.Oy, beta.Sy, beta.Oz, beta.Sz, sum);
//    }
//    // 将校准结果写入 caliVal
//    caliVal->Ox = beta.Ox;
//    caliVal->Sx = beta.Sx;
//    caliVal->Oy = beta.Oy;
//    caliVal->Sy = beta.Sy;
//    caliVal->Oz = beta.Oz;
//    caliVal->Sz = beta.Sz;
//    printf("Calibration finished!\r\n");
//    printf("Ox = %f, Sx = %f, Oy = %f, Sy = %f, Oz = %f, Sz = %f\r\n", 
//            caliVal->Ox, caliVal->Sx, caliVal->Oy, caliVal->Sy, caliVal->Oz, caliVal->Sz);
//}