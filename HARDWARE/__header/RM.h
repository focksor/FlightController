#ifndef __RM_H
#define __RM_H

#include "config.h"
#include "lcd.h"
#include "MOTOR.h"
#include "stdlib.h"
#include "stm32f10x.h"

extern unsigned char CH_Def;

void RM_Init(void);
unsigned char RC_Com(unsigned char,unsigned short);

#endif
