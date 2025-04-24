#ifndef __KALMAN_H
#define __KALMAN_H

#include "mpu6050.h"
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

/**
 * @brief 将原始数据转换为角度与加速度数据 并且对数据进行滤波 滤波数据写入roll_kalman pitch_kalman
 * 
 * @param NULL
 * @return void
 */
void angle_cal(void);

/**
 * @brief 对pitch进行滤波 采用扩展卡尔曼滤波方式
 * 
 * @param acc 加速度转换的pitch角度
 * @param gyro 陀螺仪X轴角速度
 * @return void
 */
void kalman_cal_pitch(float pitch, float gyro_x);

/**
 * @brief 对roll进行滤波 采用扩展卡尔曼滤波方式
 * 
 * @param acc 加速度转换的roll角度
 * @param gyro 陀螺仪Y轴角速度
 * @return void
 */
void kalman_cal_roll(float roll, float gyro_y);

#endif
