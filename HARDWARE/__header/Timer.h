#ifndef __TIMER_H
#define __TIMER_H

#include <stm32f10x.h>
#include <config.h>
#include <data.h>

void Timer_Heartbeat_Init(void);
void update_TimeSlice(short heartbeat);

extern __time_slice t_slice;
extern double       time_since_boot;

#endif
