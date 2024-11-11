# :pencil:四轴飞行器设计

## :rocket:项目进度

### :construction_worker:硬件

* [x] 转接板PCB设计与焊接

<img src="/Images/pcb.jpg"  alt="pcb" style="zoom:80%;" />

* [x] 分电板及稳压模块

|      组件       |             型号和参数              |
| :-------------: | :---------------------------------: |
|      主控       |       STM32F401RET6 （84MHz）       |
|      电调       |           好盈天行者-20A            |
|      电机       |         XXD 2212（1000KV）          |
|      电池       |       格氏电池 2200mah 30C 3S       |
| 加速度计&陀螺仪 |       MPU6050（集成在GY-86）        |
|     磁力计      |       HMC5883L（集成在GY-86）       |
|     遥控器      |   Microzone MC6C 2.4G 6通道遥控器   |
|     接收机      | Microzone MC6RE-V2 2.4G 6通道接收机 |
|      蓝牙       |            HC-05蓝牙模块            |



### :page_facing_up:软件

#### 一阶段

* [x] MPU6050加速度计、陀螺仪数据读取
* [x] HMC5886L磁力计数据读取
* [x] 串口数据通信
* [x] 蓝牙双端收发
* [x] PWM脉冲输出
* [x] 电机控制
* [x] 接收机PWM脉冲输入捕获



#### 二阶段

* [x] μC/OS-ii裸机移植
* [x] SystemView探针移植
* [x] OS可视化调试



#### 三阶段

* [x] MadgwickAHRS 9轴姿态融合
* [x] 匿名上位机通信
* [ ] PID反馈控制