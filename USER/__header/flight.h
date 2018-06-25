#ifndef __FlightCtrl_H
#define __FlightCtrl_H

#include "stm32f10x.h"
#include "sys.h"
#include "config.h"
#include "data.h"

#include "delay.h"
#include "usart.h"

#include "key.h"
#include "MOTOR.h"
#include "IIC.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "exti.h"
#include "rc_ppm.h"
#include "LED.h"
#include "Timer.h"

#include <imu.h>
#include <control.h>

void init_All(void);
void display_Debug_Info(void);

extern short heartbeat;

#endif	/* __FlightCtrl_H */
