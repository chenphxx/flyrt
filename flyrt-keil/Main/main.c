#include "stm32f4xx.h"


int main(void)
{
    // 使能 GPIOC 外设时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    // 配置 GPIOC 的 PC13 引脚为推挽输出模式
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;          // 选择PC13引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       // 设置为输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 设置为推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 设置为50MHz速率
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    // 不启用上下拉电阻
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    while (1)
    {
        // 点亮PC13上的LED，PC13输出低电平
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
        
        // 可以添加延时来控制LED亮灭
        for (volatile int i = 0; i < 1000000; i++);
        
        // 熄灭PC13上的LED，PC13输出高电平
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        
        // 延时
        for (volatile int i = 0; i < 1000000; i++);
    }
}
