/**
 * @file delay.h
 * @brief 延时函数头文件
 * 
 * @details 声明了延时函数的初始化和延时函数
 * 
 * @author chenphxx
 * @date 2025-2-24
 * @version V2025.2.24
 * @copyright Copyright (c) 2025
 */
#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

/**
 * @brief  延时函数初始化
 * 
 * @param None
 * @return void
 */
void delay_init(void);

/**
 * @brief 毫秒级延时函数
 * 
 * @param nTime 延时时间
 * @return void
 */
void delay_ms(uint32_t nTime);

/**
 * @brief 微秒级延时函数
 * 
 * @param nTime 延时时间
 * @return void
 */
void delay_us(uint32_t nTime);

#endif
