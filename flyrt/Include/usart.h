/**
 * @file usart.h
 * @brief USART通信接口
 * 
 * @details 提供了USART通信所需的一系列接口
 * 
 * @author chenphxx
 * @version V2025.1.18
 * @copyright (c) Copyright 2025
 */
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include <stdio.h>

#ifndef USART_H_
#define USART_H_

#define USART1_REC_LEN  200  // USART1接收缓冲 最大USART1_REC_LEN个字节
#define EN_USART1_RX 1  // 使能（1）/禁止（0）串口1接收

// 使用外部声明 避免编译时出现重复定义的报错 因为mian.c和usart.c都包含了usart.h
extern uint8_t USART_RX_BUF[USART1_REC_LEN];  // USART接收缓冲
extern uint16_t USART_RX_STA;  // USART接收状态标记

/**
 * @brief USART1初始化
 *
 * @param GPIOx 使用的GPIO外设
 * @param GPIO_Pin_TX TX引脚
 * @param GPIO_Pin_RX RX引脚
 * @param baudrate 波特率
 * @return void
 */
void usart1_init(uint32_t baudrate);

void _sys_exit(int);  // 避免使用半主机模式
int fputc(int, FILE*);  // 重定向printf

/**
 * @brief 串口中断函数
 * 
 * @param NULL
 * @return void
 */
void USART1_IRQHandler(void);

#endif
