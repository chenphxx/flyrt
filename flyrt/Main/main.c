#include "usart.h"
#include "delay.h"
#include "bldc.h"
#include "mpu6050.h"

uint16_t command = 0;
uint8_t id;
// 原始数据
int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
// 转换后加速度与角速度
float ax, ay, az, gx, gy, gz;
// 零偏校准值
extern float ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

int main(void)
{
    usart1_init(9600);
    delay_init();
    tim2_init();
    mpu6050_init();

    id = mpu6050_get_id();  // 获取MPU6050ID

    while (1)
    {
        // bldc_control(command);
        mpu6050_get_cdata(&ax, &ay, &az, &gx, &gy, &gz);  // 获取加速度和角速度
        printf("AX:%.2fm/s² AY:%.2fm/s² AZ:%.2fm/s²\r\nGX:%.2f°/s GY:%.2f°/s GZ:%.2f°/s\r\n\r\n", ax, ay, az, gx, gy, gz);

        if (USART_ReceiveData(USART1) == '1')
        {
            printf("2s后开始校准...\r\n\r\n");
            delay_ms(2000);
            printf("开始校准...\r\n\r\n");
            mpu6050_calibrate();  // 校准数据
            printf("校准完成\r\n\r\n");
            printf("当前偏移量为:AX:%.2f AY:%.2f AZ:%.2f GX:%.2f GY:%.2f GZ:%.2f\r\n\r\n", ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset);
        }
        delay_ms(500);
    }
}
