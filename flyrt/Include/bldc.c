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
    // TIM_OCInitStructure.TIM_Pulse = 1000;  // CCR1 调节占空比 范围5%-10% 1000-2000
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
    // TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);
    // TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);
    // TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);
}

uint16_t base_ccr = 0;  // 基准CCR值

// 开机时激活电调
void bldc_init(void)
{
    TIM2->CCR1 = PWM_MAX;
    TIM2->CCR2 = PWM_MAX;
    TIM2->CCR3 = PWM_MAX;
    TIM2->CCR4 = PWM_MAX;
}

uint8_t offset_a = 0;
uint8_t offset_b = 0;
uint8_t offset_c = 0;
uint8_t offset_d = 0;

// 根据信号控制电机
void bldc_ccr(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3, uint16_t ccr4)
{
    TIM2->CCR1 = ccr1;
    TIM2->CCR2 = ccr2;
    TIM2->CCR3 = ccr3;
    TIM2->CCR4 = ccr4;
}
