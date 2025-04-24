#include "usart.h"
#include "delay.h"
#include "mpu6050.h"
#include "bldc.h"
#include "pid.h"
#include "tool.h"
#include "filter.h"

/**
 * @brief 根据接收到的信号控制飞行器
 * 
 * @param NULL
 * @return void
 */
void bldc_control(void);

extern float ekf_roll_cor, ekf_pitch_cor;  // 扩展卡尔曼滤波 去除偏移量的值

extern uint16_t base_ccr;  // 初始CCR值

// CCR偏移量
extern uint8_t offset_a;
extern uint8_t offset_b;
extern uint8_t offset_c;
extern uint8_t offset_d;

extern float angle_pitch;  // 目标pitch角度
extern float angle_roll;  // 目标roll角度

int main(void)
{
    usart1_init(9600);
    tim2_init();  // 初始化PWM
    mpu6050_init(GPIOB, GPIO_Pin_6, GPIO_Pin_7);

	bldc_init();  // 将CCR置为2000 等待确定行程
	
    // 开始前校准坐标
    printf("开始校准...请等待校准完成信号\r\n\r\n");
	ekf_calibrate();  // 适用扩展卡尔曼滤波
    printf("校准完成\r\n\r\n");

    while (1)
    {
        ekf_exec();  // 扩展卡尔曼滤波
		bldc_control();  // 接收并处理控制信号
        // attitude_control(ekf_pitch_cor, ekf_roll_cor, angle_pitch, angle_roll);
    }
}

// 根据信号控制飞行器
void bldc_control(void)
{
    uint16_t command = USART_ReceiveData(USART1);
    if (command == '1')
    {
        if (base_ccr < 1100)
        {
            base_ccr = 1100;
            printf("不能小于1100\r\n");
        }
        base_ccr += 1;
        bldc_ccr(base_ccr, base_ccr, base_ccr, base_ccr);  // 调节电机的CCR值
        printf("%d %d %d %d\n", TIM2->CCR1, TIM2->CCR2, TIM2->CCR3, TIM2->CCR4);
    }
    else if (command == '2')
    {
        if (base_ccr < 1100)
        {
            base_ccr = 1100;
            printf("不能小于1100\r\n");
        }
        base_ccr -= 1;
        bldc_ccr(base_ccr, base_ccr, base_ccr, base_ccr);
        printf("%d %d %d %d\n", TIM2->CCR1, TIM2->CCR2, TIM2->CCR3, TIM2->CCR4);
    }
    else if (command == '3')  // 重置CCR offset base_ccr
    {
        TIM2->CCR1 = PWM_MIN;
        TIM2->CCR2 = PWM_MIN;
        TIM2->CCR3 = PWM_MIN;
        TIM2->CCR4 = PWM_MIN;
        offset_a = 0;
        offset_b = 0;
        offset_c = 0;
        offset_d = 0;
        base_ccr = 1000;
        printf("%d %d %d %d\n", TIM2->CCR1, TIM2->CCR2, TIM2->CCR3, TIM2->CCR4);
    }
	// 控制飞行器飞行
    else if (command == '4')  // 悬空
    {
        angle_pitch = 0.0f;
        angle_roll = 0.0f;
        attitude_control(ekf_pitch_cor, ekf_roll_cor, angle_pitch, angle_roll);
    }
    else if (command == '5')  // 前进
    {
        angle_pitch = -5.0f;
        angle_roll = 0.0f;
        attitude_control(ekf_pitch_cor, ekf_roll_cor, angle_pitch, angle_roll);
    }
    else if (command == '6')  // 后退
    {
        angle_pitch = 5.0f;
        angle_roll = 0.0f;
        attitude_control(ekf_pitch_cor, ekf_roll_cor, angle_pitch, angle_roll);
    }
    else if (command == '7')  // 向左
    {
        angle_pitch = 0.0f;
        angle_roll = 5.0f;
        attitude_control(ekf_pitch_cor, ekf_roll_cor, angle_pitch, angle_roll);
    }
    else if (command == '8')  // 向右
    {
        angle_pitch = 0.0f;
        angle_roll = -5.0f;
        attitude_control(ekf_pitch_cor, ekf_roll_cor, angle_pitch, angle_roll);
    }
    else if (command == '9')  // 获取当前角度数据
    {
        printf("%.2f, %.2f\r\n", ekf_roll_cor, ekf_pitch_cor);
    }
    else if (command == '0')
    {
        bldc_ccr(base_ccr, base_ccr, base_ccr, base_ccr);
        printf("%d %d %d %d\n", TIM2->CCR1, TIM2->CCR2, TIM2->CCR3, TIM2->CCR4);
    }
}
