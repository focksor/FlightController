#ifndef __RM_H
#define __RM_H

#include "config.h"
#include "delay.h"
#include "MOTOR.h"
#include "stdlib.h"
#include "stm32f10x.h"

void RC_PPM_Init(void);
void refine_RC_PPM_Data(void);

extern __rc_channel rc, rc_last;

#endif
