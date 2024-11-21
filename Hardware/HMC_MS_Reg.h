#ifndef _HMC_MS_REG_H_
#define _HMC_MS_REG_H_


#define HMC5883L_Address 					0x1E
//#define HMC5883L_Address 					0x3C

#define HMC5883L_Config_A 					0x00
#define HMC5883L_Config_B 					0x01
#define HMC5883L_Mode 						0x02
#define	HMC5883L_MAGNETIC_XOUT_H			0x03
#define	HMC5883L_MAGNETIC_XOUT_L			0x04
#define	HMC5883L_MAGNETIC_YOUT_H			0x05
#define	HMC5883L_MAGNETIC_YOUT_L			0x06
#define	HMC5883L_MAGNETIC_ZOUT_H			0x07
#define	HMC5883L_MAGNETIC_ZOUT_L			0x08




#define MS561101BA_Addr 0xEE //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)
#define MS561101BA_Addr_Real 0x77

// 定义MS561101BA内部地址
// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E
// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3
// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08
//#define  MS561101BA_D1_OSR_256 0x40
//#define  MS561101BA_D1_OSR_512 0x42
//#define  MS561101BA_D1_OSR_1024 0x44
//#define  MS561101BA_D1_OSR_2048 0x46
#define MS561101BA_D1_OSR_4096 0x48
//#define  MS561101BA_D2_OSR_256 0x50
//#define  MS561101BA_D2_OSR_512 0x52
//#define  MS561101BA_D2_OSR_1024 0x54
//#define  MS561101BA_D2_OSR_2048 0x56
#define MS561101BA_D2_OSR_4096 0x58
#define MS561101BA_PROM_BASE_ADDR 0xA0 // by adding ints from 0 to 6 we can read all the prom configuration values.
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.

#endif
