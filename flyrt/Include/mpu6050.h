/**
 * @file mpu6050.h
 * @brief 姿态传感器文件
 * 
 * @details 配置MPU6050的通信与数据读取
 * 
 * @author chenphxx
 * @date 2025-3-26
 * @version V2025.3.26
 */
#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx.h"

// 地址配置 
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

// 偏移量
extern float ax_offset, ay_offset, az_offset;
extern float gx_offset, gy_offset, gz_offset;

/**
 * @brief  MPU6050初始化
 *
 * @param NULL
 * @return void
 */
void mpu6050_init(void);

/**
 * @brief  MPU6050等待事件
 *
 * @param i2cx I2C外设
 * @param i2c_event 要等待的I2C事件
 * @return void
 */
void mpu6050_wait_event(I2C_TypeDef *i2cx, uint32_t i2c_event);

/**
 * @brief  MPU6050写寄存器
 *
 * @param reg_address 寄存器地址
 * @param data 要写入的寄存器数据
 * @return void
 */
void mpu6050_write_reg(uint8_t reg_address, uint8_t data);

/**
 * @brief  MPU6050读寄存器
 *
 * @param reg_address 寄存器地址
 * @return 读取到的寄存器数据
 */
uint8_t mpu6050_read_reg(uint8_t reg_address);

/**
 * @brief  获取MPU6050的ID
 *
 * @param NULL
 * @return MPU6050的ID
 */
uint8_t mpu6050_get_id(void);

/**
 * @brief  读取MPU6050的加速度计和陀螺仪数据
 *         读取的是原始数据 需要将其转换为四元数或欧拉角
 *
 * @param acc_x 加速度计X轴数据
 * @param acc_y 加速度计Y轴数据
 * @param acc_z 加速度计Z轴数据
 * @param gyro_x 陀螺仪X轴数据
 * @param gyro_y 陀螺仪Y轴数据
 * @param gyro_z 陀螺仪Z轴数据
 * @return void
 */
void mpu6050_get_odata(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

/**
 * @brief 获取加速度和角速度数据
 * 
 * @param ax 加速度计X轴数据
 * @param ay 加速度计Y轴数据
 * @param az 加速度计Z轴数据
 * @param gx 陀螺仪X轴数据
 * @param gy 陀螺仪Y轴数据
 * @param gz 陀螺仪Z轴数据
 * @return void
 */
void mpu6050_get_cdata(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

/**
 * @brief 零偏校准器 获取校准后的加速度与角速度数据; 
 *        水平放置传感器 静止采集100次数据 取平均值; 
 *        准确值 = 原始值 - 水平静置传感器收集100次数据的平均值; 
 *        当前并没有对Z轴进行校准 因为Z轴的数据大致等于1g 属于正确数据
 * 
 * @return void
 */
void mpu6050_calibrate(void);

#endif
