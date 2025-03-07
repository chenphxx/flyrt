#include "bldc.h"

// 定时器2初始化 通过四个通道输出PWM信号
void tim2_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // CH1-CH4 PA0-PA3
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 设置复用功能为TIM2定时器的四个通道
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

    // 配置TIM2
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;  // PSC 预分频系数
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // 向上计数
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1;  // ARR 自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // 时钟分频因子
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);  // 初始化TIM2

    TIM_Cmd(TIM2, ENABLE);  // 使能TIM2

    // 配置TIM2的通道1
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  // 输出使能
    TIM_OCInitStructure.TIM_Pulse = 1000;  // CCR1 调节占空比 范围5%-11% 1000-2200
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  // 输出极性

    // 初始化定时器通道
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);

    // 使能预装载寄存器 当CCR发生改变时 不会立即影响占空比 而是会等待CNT达到ARR时才发生改变 防止PWM跳变
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    // 关闭预装载寄存器
    // TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
}

// 根据信号控制电机
void bldc_control(uint16_t command)
{
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)  // 检测是否有数据
    {
        command = USART_ReceiveData(USART1);
        // CCR1
        if (command == '1')  // 增加占空比 每次增加0.1%
        {
            if (TIM2->CCR1 + 20 <= TIM2->ARR)
            {
                TIM2->CCR1 += 20;
            }
        }
        else if (command == '2')  // 减小占空比 每次减少0.1%
        {
            if (TIM2->CCR1 >= 1000)
            {
                TIM2->CCR1 -= 20;
            }
        }
        // CCR2
        else if (command == '3')
        {
            if (TIM2->CCR2 + 20 <= TIM2->ARR)
            {
                TIM2->CCR2 += 20;
            } 
        }
        else if (command == '4')
        {
            if (TIM2->CCR2 >= 1000)
            {
                TIM2->CCR2 -= 20;
            }
        }
        // CCR3
        else if (command == '5')
        {
            if (TIM2->CCR3 + 20 <= TIM2->ARR)
            {
                TIM2->CCR3 += 20;
            } 
        }
        else if (command == '6')
        {
            if (TIM2->CCR3 >= 1000)
            {
                TIM2->CCR3 -= 20;
            }
        }
        // CCR4
        else if (command == '7')
        {
            if (TIM2->CCR4 + 20 <= TIM2->ARR)
            {
                TIM2->CCR4 += 20;
            } 
        }
        else if (command == '8')
        {
            if (TIM2->CCR4 >= 1000)
            {
                TIM2->CCR4 -= 20;
            }
        }
        // 复位占空比 恢复5%占空比
        else if (command == '5')
        {
            TIM2->CCR1 = 1000;
            TIM2->CCR2 = 1000;
            TIM2->CCR3 = 1000;
            TIM2->CCR4 = 1000;
        }

        printf("%.1f%% %.1f%% %.1f%% %.1f%%\n", 
                (float)TIM2->CCR1 / (TIM2->ARR + 1) * 100, 
                (float)TIM2->CCR2 / (TIM2->ARR + 1) * 100,
                (float)TIM2->CCR3 / (TIM2->ARR + 1) * 100,
                (float)TIM2->CCR4 / (TIM2->ARR + 1) * 100);
    }
}
