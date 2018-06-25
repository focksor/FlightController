#ifndef __CONFIGURE_H
#define __CONFIGURE_H

#include <sys.h>
#include <usart.h>

#define PI 3.1415926535897932384626

typedef enum {false = 0,true = 1} bool;

typedef enum {
    roll_ch     = 1,
    pitch_ch    = 2,
    throttle_ch = 3,
    yaw_ch      = 4,
    sw_5_ch     = 5,
    sw_6_ch     = 6,
    sw_7_ch     = 7,
    sw_8_ch     = 8,
    sw_9_ch     = 9,
    sw_10_ch    = 10,
} __rc_info_t;


typedef enum {
    mode_stabilized = 1,
    mode_acro       = 2,
} __flight_mode_t;

typedef struct __v3f_ {
    float x;
    float y;
    float z;
    
} __vector3f;

typedef struct __imu_ {
    float pitch;
    float roll;
    float yaw;
} __imu;

typedef struct {
    short pitch;
    short roll;
    short throttle;
    short yaw;
    short ch_5;
    short ch_6;
    short ch_7;
    short ch_8;
    short ch_9;
    short ch_10;
} __rc_channel;

typedef struct {
    bool _200Hz;
    bool _100Hz;
    bool _50Hz;
    bool _25Hz;
    bool _10Hz;
    bool _5Hz;
    bool _1Hz;
} __time_slice;

//__MOTOR_DEFINE
#define __MOTOR_Pin (GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9)//GPIO_Pin of motor pin,GPIOB

//__MPU9250_DEFINE
//!!__CHOOSE GPIO PIN IN GPIO B
#define __IIC_SCL_Pin GPIO_Pin_12
#define __IIC_SDA_Pin GPIO_Pin_13
#define __MPU9250_Pin (__IIC_SCL_Pin | __IIC_SDA_Pin)
#define __IIC_Pin __MPU9250_Pin

/*
REMOTE_CONTROLLER
PPM PROGRAM SET IN EXTI.C
REMOTE CONTROLLER PROGRAM SET IN RM.C
*/
#define __RM_Pin GPIO_Pin_6//GPIO A


#endif	/* __CONFIGURE_H */

