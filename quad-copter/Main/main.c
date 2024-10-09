#include "stm32f10x.h"


// 控制两个LED灯
int main(void)
{
	// GPIO初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // 设置推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;  // 初始化引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	while (1)
	{
		// LED开
		GPIO_SetBits(GPIOA, GPIO_Pin_0);
		// GPIO_SetBits(GPIOA, GPIO_Pin_1);
		GPIO_SetBits(GPIOA, GPIO_Pin_2);
		
		// Delay_ms(500)
		// GPIO_ResetBits(GPIOA, GPIO_Pin_0);  // 关灯
	}
}
