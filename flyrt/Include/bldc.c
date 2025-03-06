#include "bldc.h"

// 定时器2初始化
void tim2_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // PA5
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2);  // 复用为TIM2

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
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);  // 初始化TIM2的通道1 OC1表示CH1
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  // 使能TIM2在CCR1上的预装载寄存器
}

// 根据信号控制电机
void bldc_control(uint16_t command)
{
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)  // 检测是否有数据
    {
        command = USART_ReceiveData(USART1);
        if (command == '3')  // 增加占空比 每次增加0.1%
        {
            if (TIM2->CCR1 + 20 <= TIM2->ARR)
            {
                TIM2->CCR1 += 20;
            }
        }
        else if (command == '4')  // 减小占空比 每次减少0.1%
        {
            if (TIM2->CCR1 >= 1000)
            {
                TIM2->CCR1 -= 20;
            }
        }
        else if (command == '5')  // 复位占空比 恢复5%占空比
        {
            TIM2->CCR1 = 1000;
        }

        printf("占空比: %.2f%%\n", (float)TIM2->CCR1 / (TIM2->ARR + 1) * 100);
    }
}
