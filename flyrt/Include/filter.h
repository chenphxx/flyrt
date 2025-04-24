/**
 * @file filter.h
 * @brief 数据滤波文件
 * 
 * @details 对采集到的姿态数据进行数据滤波
 * 
 * @author chenphxx
 * @date 2025-4-8
 * @version V2025.4.8
 */
#ifndef __FILTER_H
#define __FILTER_H

#include "mpu6050.h"
#include "usart.h"
#include "math.h"

extern short temperature;  // 陀螺仪温度数据
extern float accel_x;  // X轴加速度值
extern float accel_y;  // Y轴加速度值
extern float accel_z;  // Z轴加速度值
extern float gyro_x;  // X轴陀螺仪角速度数据
extern float gyro_y;  // Y轴陀螺仪角速度数据
extern float gyro_z;  // Z轴陀螺仪角速度数据

extern float yaw_raw;  // 航线角yaw原始数据
extern float yaw_kalman;  // yaw滤波后数据

extern float pitch_raw;  // 俯仰角pitch原始数据
extern float pitch_kalman;  // pitch滤波后数据
extern float roll_raw;  // 横滚角roll原始数据
extern float roll_kalman;  // roll滤波后数据

extern short temp;  // 温度

extern float ekf_roll_offset;  // roll偏移量
extern float ekf_pitch_offset;  // pitch偏移量

/**
 * @brief 对pitch进行滤波 采用扩展卡尔曼滤波方式
 * 
 * @param acc 加速度转换的pitch角度
 * @param gyro 陀螺仪X轴角速度
 * @return void
 */
void ekf_pitch(float pitch, float gyro_x);

/**
 * @brief 对roll进行滤波 采用扩展卡尔曼滤波方式
 * 
 * @param acc 加速度转换的roll角度
 * @param gyro 陀螺仪Y轴角速度
 * @return void
 */
void ekf_roll(float roll, float gyro_y);

/**
 * @brief 执行扩展卡尔曼滤波
 * 
 * @param NULL
 * @return void
 */
void ekf_exec(void);

/**
 * @brief 校准初始角度 适用扩展卡尔曼滤波
 * 必须保留最后的printf语句 否则会导致编译器将pitch偏移量优化掉 目前还没有找到问题原因 等待后续重构函数
 * 
 * @param NULL
 * @return void
 */
void ekf_calibrate(void);

#endif
