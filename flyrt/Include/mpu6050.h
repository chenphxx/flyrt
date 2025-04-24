#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx.h"
#include "iic.h"
#include "math.h"

extern struct I2C_BUS MPU6050_I2C;

#define MPU6050_ADDRESS 0x68  // i2c address
#define MPU6050_SMPLRT_DIV 0x19  // 采样率
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_FIFO_EN 0x23

#define MPU6050_INTBP_CFG_REG 0X37 // 中断寄存器
#define MPU6050_INT_ENABLE 0x38

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_SIGNAL_PATH_RESET 0x68

#define MPU6050_USER_CTRL 0x6A
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHO_AM_I 0x75

/**
 * @brief 原始数据结构体
 */
typedef struct MPU6050_Raw
{
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
    uint16_t Temp;
} MPU6050_Raw;

/**
 * @brief 角度数据结构体
 */
typedef struct MPU6050
{
    float yaw;
    float roll;
    float pitch;
} MPU6050;

/**
 * @brief 获取MPU6050 ID
 * 
 * @param NULL
 * @return uint8_t ID
 */
uint8_t mpu6050_id(void);  // 读取ID

/**
 * @brief 初始化MPU6050
 * 
 * @param GPIOx 
 * @param SCL
 * @param SDA
 * @return void
 */
void mpu6050_init(GPIO_TypeDef *GPIOx, uint16_t SCl, uint16_t SDA);

/**
 * @brief 获取原生数据
 * 
 * @param this
 * @return void
 */
void mpu6050_get_raw(MPU6050_Raw *this);

/**
 * @brief 获取角度数据 这个步骤会对角度进行互补滤波处理
 * 
 * @param this
 * @return void
 */
void mpu6050_get_angle(MPU6050 *this);

/**
 * @brief 获取温度
 * 
 * @param NULL
 * @return float 温度
 */
float mpu6050_get_temp(void);

/**
 * @brief 校准初始角度 针对一维卡尔曼滤波实现
 * 
 * @param NULL
 * @return void
 */
void mpu6050_calibrate(void);

#endif
