#include "delay.h"

// 使用基本定时器实现延时
void delay_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);  // 使能TIM6时钟
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 84 - 1;  // 预分频系数 84M/(84-1+1)=1M 每秒计数1M次 即1us计数一次
    TIM6->ARR = 65536 - 1;  // 自动重装载值 即计数到65535后重新计数
    TIM6->CR1 |= TIM_CR1_CEN; // 启动定时器
}

// 微秒级延时函数
void delay_us(uint32_t time)
{
    TIM6->CNT = 0; // 计数器清零
    // 确保时间不超过 ARR
    if (time <= 65535)
    {
        while (TIM6->CNT < time); // 循环直到计数器达到设定时间
    }
}

void delay_ms(uint32_t time)
{
    // 直接使用delay_us来实现毫秒级延时
    while (time--)
    {
        delay_us(1000);
    }
}
