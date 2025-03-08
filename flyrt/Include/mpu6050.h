/**
 * @file mpu6050.h
 * @brief MPU6050驱动文件
 * 
 * @details 使用硬件IIC方式驱动MPU6050
 *          SCL-PB5 SDA-PB6
 * 
 * @author chenphxx
 * @date 2025-3-8
 * @version V2025.3.8
 */
#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx.h"

#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

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

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_WHO_AM_I 0x75

/**
 * @brief  MPU6050初始化
 *
 * @param None
 * @return void
 */
void MPU6050_Init(void);

/**
 * @brief  MPU6050等待事件
 *
 * @param I2Cx I2C外设
 * @param I2C_EVENT 要等待的I2C事件
 * @return void
 */
void MPU6050_WaitEvent(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT);

/**
 * @brief  MPU6050写寄存器
 *
 * @param RegAddress 寄存器地址
 * @param Data 要写入的寄存器数据
 * @return void
 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);

/**
 * @brief  MPU6050读寄存器
 *
 * @param RegAddress 寄存器地址
 * @return 读取到的寄存器数据
 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

/**
 * @brief  获取MPU6050的ID号
 *
 * @param None
 * @return MPU6050的ID号
 */
uint8_t MPU6050_GetID(void);

/**
 * @brief  获取MPU6050的加速度计和陀螺仪数据
 *
 * @param AccX 加速度计X轴数据
 * @param AccY 加速度计Y轴数据
 * @param AccZ 加速度计Z轴数据
 * @param GyroX 陀螺仪X轴数据
 * @param GyroY 陀螺仪Y轴数据
 * @param GyroZ 陀螺仪Z轴数据
 * @return void
 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif
