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

#define PWM_MAX 2000
#define PWM_MIN 1000

extern uint16_t base_ccr;  // 占空比

/**
 * @brief 通用定时器2初始化
 * 
 * @param NULL
 * @return void
 */
void tim2_init(void);

/**
 * @brief 在开机时设定电机转速 需要接一个最小转速1000CCR
 * 
 * @param NULL
 * @return void
 */
void bldc_init(void);

/**
 * @brief 调节四个电机的CCR
 * 
 * @param ccr1 电机1的CCR值
 * @param ccr2 电机2的CCR值
 * @param ccr3 电机3的CCR值
 * @param ccr4 电机4的CCR值
 * @return void
 */
void bldc_ccr(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3, uint16_t ccr4);

#endif
