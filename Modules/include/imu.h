#ifndef __IMU_H
#define __IMU_H

#include <stm32f10x.h>

#include <config.h>
#include <data.h>

#include <mpu6050.h>
#include <inv_mpu.h>
#include <filter.h>

#define  ACC_IIR_ORDER     4      //使用IIR滤波器的阶数

bool update_IMU_Data(void);
void calc_Gyro_Offset(void);

void fix_IMU_Orentation(void);

extern __imu IMU, IMU_last;
extern __imu imu_dmp;

extern __vector3f acc,  acc_last;        // m * s^-2
extern __vector3f gyro, gyro_last;       // dps, degree per second
extern __vector3f gyro_idle;

#endif  // __IMU_H
