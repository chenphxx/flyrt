/**
 * @file delay.h
 * @brief 延时函数头文件
 * 
 * @details 声明了延时函数的初始化和延时函数
 * 
 * @author chenphxx
 * @date 2025-4-28
 * @version V2025.4.28
 * @copyright Copyright (c) 2025
 */
#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

#define PC13(x) GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)x)

/**
 * @brief 100纳秒级延时函数
 * 
 * @param n 延时时间
 * @return void
 */
void delay_100ns(uint32_t n);

/**
 * @brief 微秒级延时函数
 * 
 * @param us 延时时间
 * @return void
 */
void delay_us(uint32_t us);

/**
 * @brief 毫秒级延时函数
 * 
 * @param ms 延时时间
 * @return void
 */
void delay_ms(uint32_t ms);

/**
 * @brief 秒级延时函数
 * 
 * @param s 延时时间
 * @return void
 */
void delay_s(uint32_t s);

#endif
