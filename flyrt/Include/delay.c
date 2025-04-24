#include "delay.h"

// 延时 100ns，使用 TIM2 定时器进行精确计时
void delay_100ns(uint32_t n)
{
    // 使用 TIM2 计时器，假设 TIM2 时钟频率为 180 MHz (适应 STM32F4)
    TIM_SetCounter(TIM2, 0);
    while (n > 0)
    {
        while (TIM_GetCounter(TIM2) < 1)
            ; // 等待定时器计数器值达到 1
        TIM_SetCounter(TIM2, 0);
        n--;
    }
}

// 微秒延时
void delay_us(uint32_t us)
{
    if (us == 0)
        return;
    // STM32F4 系列的 SysTick 使用 168MHz 系统时钟
    SysTick->LOAD = 168 * us - 1;  // 设置负载值
    SysTick->VAL = 0x00;  // 清空计数器
    SysTick->CTRL = 0x00000005;  // 启动 SysTick 使用系统时钟 启用计数器
    while (!(SysTick->CTRL & 0x00010000))  // 等待计时结束
        ;
    SysTick->CTRL = 0x00000004;  // 禁用 SysTick
}

// 毫秒延时
void delay_ms(uint32_t xms)
{
    while (xms--)
        delay_us(1000);
}

// 秒级延时
void delay_s(uint32_t xs)
{
    while (xs--)
        delay_ms(1000);
}
