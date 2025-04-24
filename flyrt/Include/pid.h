/**
 * @file pid.h
 * @brief 姿态传感器文件
 * 
 * @details PID算法的实现
 *          飞行器动力设计分配如下
 * 
 *          B(逆时针)        A(顺时针)
 *                     
 *                                   ⬆(机头方向)
 * 
 *          C(顺时针)        D(逆时针)
 * 
 * @author chenphxx
 * @date 2025-4-13
 * @version V2025.4.13
 */
#ifndef __PID_H
#define __PID_H

#include "bldc.h"
#include "mpu6050.h"
#include "usart.h"

extern float angle_roll;  // 目标roll
extern float angle_pitch;  // 目标pitch

/**
 * @brief PID 控制器结构体
 */
typedef struct
{
    float kp, ki, kd;

    float setpoint;
    float measured;
    float error;
    float last_error;
    float integral;
    float derivative;
    float output;

    float output_max; // 输出限幅
    float output_min;

    float integral_max; // 积分限幅
    float integral_min;

    float integral_separation_threshold; // 积分分离门限
} PID_Controller_t;

/**
 * @brief 初始化PID控制器
 * 
 * @param pid PID控制器指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param out_min 输出最小值
 * @param out_max 输出最大值
 */
void pid_init(PID_Controller_t *pid, float kp, float ki, float kd, float out_min, float out_max);

/**
 * @brief 重置PID控制器内部状态(积分项和误差)
 * 
 * @param pid PID控制器指针
 */
void pid_reset(PID_Controller_t *pid);

/**
 * @brief 计算 PID 输出
 * 
 * @param pid PID控制器指针 
 * @param setpoint 目标姿态角度
 * @param measured 实际姿态角度
 * @return float PID输出结果
 */
float pid_compute(PID_Controller_t *pid, float setpoint, float measured);

/**
 * @brief 姿态控制主函数（应定时调用）
 * 
 * @param pitch 当前pitch角
 * @param roll 当前roll角
 * @param angle_pitch 目标pitch
 * @param angle_roll 目标roll
 * @return void
 */
void attitude_control(float pitch, float roll, float angle_pitch, float angle_roll);

#endif
