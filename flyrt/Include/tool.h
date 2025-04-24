/**
 * @file tool.h
 * @brief 一些用得上的工具
 * 
 * @details 用于互补滤波时的排序
 * 
 * @author chenphxx
 * @date 2025-4-5
 * @version V2025.4.7
 */
#ifndef __TOOL_H
#define __TOOL_H

#include "stm32f4xx.h"

#define DEG_TO_RAD 0.017453293f  // 角度转换到弧度（π/180）

// 设置量程
#define FS_SEL 3

#if FS_SEL == 0  // ±250°/s
    #define RawData_to_Angle 0.007633f  // 1/131 ≈ 0.007633 °/s per LSB
    #define RawData_to_Radian (DEG_TO_RAD * RawData_to_Angle)  // ≈ 0.0001332 rad/LSB
#elif FS_SEL == 1  // ±500°/s
    #define RawData_to_Angle 0.015267f  // 1/65.5 ≈ 0.015267 °/s per LSB
    #define RawData_to_Radian (DEG_TO_RAD * RawData_to_Angle)  // ≈ 0.0002665 rad/LSB
#elif FS_SEL == 2  // ±1000°/s
    #define RawData_to_Angle 0.030488f  // 1/32.8 ≈ 0.030488 °/s per LSB
    #define RawData_to_Radian (DEG_TO_RAD * RawData_to_Angle)  // ≈ 0.0005369 rad/LSB
#elif FS_SEL == 3  // ±2000°/s
    #define RawData_to_Angle 0.0610351f  // 1/16.4 ≈ 0.0610351 °/s per LSB
    #define RawData_to_Radian (DEG_TO_RAD * RawData_to_Angle)  // ≈ 0.0010653 rad/LSB
#else
    #error "Invalid FS_SEL value! Must be 0, 1, 2, or 3."
#endif

#define Radian_to_Angle 57.2957795f  // 固定值

/**
 * @brief 快速计算开根号的倒数
 * 
 * @param x 要计算的变量
 * @return float 计算结果
 */
float invSqrt(float x);

/**
 * @brief 将原始值转换为弧度值
 * 
 * @param gx 陀螺仪X轴数据
 * @param gy 陀螺仪Y轴数据
 * @param gz 陀螺仪Z轴数据
 * @param gx_rad 陀螺仪X轴数据
 * @param gy_rad 陀螺仪Y轴数据
 * @param gz_rad 陀螺仪Z轴数据
 * @return void
 */
void get_radian(float *gx, float *gy, float *gz, float *gx_rad, float *gy_rad, float *gz_rad);

/**
 * @brief 确定元素位置
 * 
 * @param arr 目标数组
 * @param low 数组最小下标
 * @param high 数组最大下标
 * @return low数组位置
 */
int partition(int16_t *arr, int low, int high);

/**
 * @brief 快速排序
 * 
 * @param arr 目标数组
 * @param low 数组最小下标
 * @param high 数组最大下标
 * @return void
 */
void quick_sort(int16_t *arr, int low, int high);

#endif
