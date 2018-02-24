#ifndef __CONFIGURE_H
#define __CONFIGURE_H

typedef enum {FALSE = 0,TRUE = 1} bool;

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

