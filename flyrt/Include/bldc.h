/**
 * @file bldc.h
 * @brief 无刷电机头文件
 * 
 * @details 通过定时器向电调输出PWM信号进行控制, 改变占空比控制转速
 *          占空比为4%-5.1%表示最低转速, 5.2%电机开始转动
 *          占空比11%表示最大转速, 占空比达到11.8%时电机停止工作
 * 
 * @author chenphxx
 * @date 2025-3-6
 * @version V2025.3.6
 */
#ifndef __BLDC_H
#define __BLDC_H

#include "usart.h"

/**
 * @brief 通用定时器2初始化
 * 
 * @param NULL
 * @return void
 */
void tim2_init();

/**
 * @brief 根据信号控制电机
 * 
 * @param command 上位机发送的控制信号
 * @return void
 */
void bldc_control(uint16_t command);

#endif
