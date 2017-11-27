#ifndef __CONFIGURE_H

#define __CONFIGURE_H


//__MOTOR_DEFINE
#define __MOTOR_Pin (GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9)//GPIO_Pin of motor pin,GPIOB

//__MPU9250_DEFINE
//!!__CHOOSE GPIO PIN IN GPIO B
#define __IIC_SCL_Pin GPIO_Pin_6
#define __IIC_SDA_Pin GPIO_Pin_7
#define __MPU9250_Pin (__IIC_SCL_Pin | __IIC_SDA_Pin)
#define __IIC_Pin __MPU9250_Pin



#endif	/* __CONFIGURE_H */

