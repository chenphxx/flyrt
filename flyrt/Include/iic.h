#ifndef __IIC__H_
#define __IIC__H_

#include "stm32f4xx.h"
#include "delay.h"

#define SI2C_delay_time 0  // 设置硬件I2C的延时速度
#define I2C(obj)	(Pthis = &obj)  // Pthis全局指针宏定义
typedef struct I2C_Private I2C_Private;
typedef struct I2C_BUS
{
    // 严禁使用该指针
    I2C_Private *Private;
    uint8_t (*AckTest)();  // 响应接口
    uint8_t (*Read_Reg)(uint8_t RegAddress);  // 读寄存器函数
    
    void (*Write_Reg)(uint8_t RegAddress, uint8_t Data);  // 写寄存器函数
    
    void (*Rest_Speed)(uint32_t Speed);  // 硬件I2C重新设置速度

    uint8_t (*Write_Reg_continue)(uint8_t Device_Add, uint8_t RegAddress, uint8_t Count, uint8_t *Data); // 连续写寄存器函数 为了适应移植
    uint8_t (*Read_Reg_continue)(uint8_t Device_Add, uint8_t RegAddress, uint8_t Count, uint8_t *Data);  // 连续读寄存器函数 为了适应移植
} I2C_BUS;

extern I2C_BUS *Pthis;

I2C_BUS Create_SI2C(GPIO_TypeDef *GPIOx, uint16_t SCL, uint16_t SDA, uint8_t Address);  // 创建软件I2C对象,create a softwere I2C

#endif
