#include "usart.h"
#include "delay.h"
#include "bldc.h"
#include "mpu6050.h"

int main(void)
{
    usart1_init(9600);
    delay_init();
    tim2_init();

    uint16_t command = 0;

    while (1)
    {
        bldc_control(command);
    }
}
