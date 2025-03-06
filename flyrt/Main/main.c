#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "tim.h"

int main(void)
{
    delay_init();
	usart1_init(9600);
	tim_init();
    
    while (1)
    {
		// pass
    }
}
