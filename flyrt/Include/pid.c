#include "pid.h"

// pid初始化
void pid_init(PID_Controller_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = out_min;
    pid->output_max = out_max;

    pid_reset(pid);
}

// pid重置
void pid_reset(PID_Controller_t *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
}

// pid计算
float pid_compute(PID_Controller_t *pid, float setpoint, float measured)
{
    pid->setpoint = setpoint;
    pid->measured = measured;

    pid->error = setpoint - measured;

    // 积分分离 大误差时不积分
    if (fabsf(pid->error) < pid->integral_separation_threshold)
	{
        pid->integral += pid->error;

        // 积分限幅
        if (pid->integral > pid->integral_max)
            pid->integral = pid->integral_max;
        else if (pid->integral < pid->integral_min)
            pid->integral = pid->integral_min;
    }
	else
	{
        // 大误差时冻结积分 防止积分累积误修正
        pid->integral = 0;
    }

    // 微分项 当前误差与上一误差差值
    pid->derivative = pid->error - pid->last_error;

    // PID公式
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;

    // 输出限幅
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < pid->output_min)
        pid->output = pid->output_min;

    pid->last_error = pid->error;

    return pid->output;
}

// pid控制器
PID_Controller_t pid_roll, pid_pitch;

float angle_pitch = 0;
float angle_roll = 0;

// 姿态控制
void attitude_control(float pitch, float roll, float angle_pitch, float angle_roll)
{
    pid_init(&pid_pitch, 3.0f, 0.01f, 1.2f, -200, 200);  // Pitch PID
    pid_init(&pid_roll,  3.0f, 0.01f, 1.2f, -200, 200);  // Roll PID

    float pitch_output = pid_compute(&pid_pitch, angle_pitch, pitch);  // 目标为0°
    float roll_output  = pid_compute(&pid_roll, angle_roll, roll);   // 目标为0°

    float motor_a = base_ccr + pitch_output + roll_output;  // 右上角 电机A
    float motor_b = base_ccr + pitch_output - roll_output;  // 左上角 电机B
    float motor_c = base_ccr - pitch_output - roll_output;  // 左下角 电机C
    float motor_d = base_ccr - pitch_output + roll_output;  // 右下角 电机D

    // 限幅 防止超过最大和最小
    if (motor_a > PWM_MAX) motor_a = PWM_MAX;
    if (motor_a < PWM_MIN) motor_a = PWM_MIN;
    
    if (motor_b > PWM_MAX) motor_b = PWM_MAX;
    if (motor_b < PWM_MIN) motor_b = PWM_MIN;
    
    if (motor_c > PWM_MAX) motor_c = PWM_MAX;
    if (motor_c < PWM_MIN) motor_c = PWM_MIN;
    
    if (motor_d > PWM_MAX) motor_d = PWM_MAX;
    if (motor_d < PWM_MIN) motor_d = PWM_MIN;

    bldc_ccr(motor_a, motor_b, motor_c, motor_d);  // 调节电机的CCR值
    printf("\r\n");
    printf("[B: %d]   [A: %d]\r\n", TIM2->CCR2, TIM2->CCR1);
    printf("\r\n\r\n");
    printf("[C: %d]   [D: %d]\r\n", TIM2->CCR3, TIM2->CCR4);
    printf("\r\n\r\n");
}
