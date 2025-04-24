#include "kalman.h"

float accel_x;  // X轴加速度值
float accel_y;  // Y轴加速度值
float accel_z;  // Z轴加速度值
float gyro_x;  // X轴陀螺仪角速度数据
float gyro_y;  // Y轴陀螺仪角速度数据
float gyro_z;  // Z轴陀螺仪角速度数据

float yaw_raw;  // 航线角yaw原始数据
float yaw_kalman;  // yaw滤波后数据

float pitch_raw;  // 俯仰角pitch原始数据
float pitch_kalman;  // pitch滤波后数据
float roll_raw;  // 横滚角roll原始数据
float roll_kalman;  // roll滤波后数据

short temp;  // 温度

// 陀螺仪数据计算
void angle_cal(void)
{
    // 原始数据获取
    float accx, accy, accz;  // 三方向角加速度值
	MPU6050_raw ret;
    
	temp = mpu6050_get_temp();  // 获取温度
    mpu6050_get_raw(&ret);  // 获取原始数据

    accel_x = (float)ret.AccX;  // x轴加速度值暂存
    accel_y = (float)ret.AccY;  // y轴加速度值暂存
    accel_z = (float)ret.AccZ;  // z轴加速度值暂存
    gyro_x = (float)ret.GyroX;  // x轴陀螺仪值暂存
    gyro_y = (float)ret.GyroY;  // y轴陀螺仪值暂存
    gyro_z = (float)ret.GyroZ;  // z轴陀螺仪值暂存

    // 角加速度原始值处理
    // 加速度传感器配置寄存器0X1C内写入0x01,设置范围为±2g。换算关系：2^16/4 = 16384LSB/g
    if (accel_x < 32764)
        accx = accel_x / 16384.0; // 计算x轴加速度
    else
        accx = 1 - (accel_x - 49152) / 16384.0;
    if (accel_y < 32764)
        accy = accel_y / 16384.0; // 计算y轴加速度
    else
        accy = 1 - (accel_y - 49152) / 16384.0;
    if (accel_z < 32764)
        accz = accel_z / 16384.0; // 计算z轴加速度
    else
        accz = (accel_z - 49152) / 16384.0;
    // 加速度反正切公式计算三个轴和水平面坐标系之间的夹角
    if (accz == 0)
    {
        pitch_raw = 1;
        roll_raw = 1;
    }
    else
    {
        pitch_raw = (atan(accy / accz)) * 180 / 3.14;
        roll_raw = (atan(accx / accz)) * 180 / 3.14;
    }

    // 判断计算后角度的正负号
    if (accel_x < 32764)
        roll_raw = +roll_raw;
    if (accel_x > 32764)
        roll_raw = -roll_raw;
    if (accel_y < 32764)
        pitch_raw = +pitch_raw;
    if (accel_y > 32764)
        pitch_raw = -pitch_raw;

    // 角速度原始值处理过程
    // 陀螺仪配置寄存器0X1B内写入0x18，设置范围为±2000deg/s。换算关系：2^16/4000=16.4LSB/(°/S)
    //  计算角速度
    if (gyro_x < 32768)
        gyro_x = -(gyro_x / 16.4);
    if (gyro_x > 32768)
        gyro_x = +(65535 - gyro_x) / 16.4;
    if (gyro_y < 32768)
        gyro_y = -(gyro_y / 16.4);
    if (gyro_y > 32768)
        gyro_y = +(65535 - gyro_y) / 16.4;
    if (gyro_z < 32768)
        gyro_z = -(gyro_z / 16.4);
    if (gyro_z > 32768)
        gyro_z = +(65535 - gyro_z) / 16.4;

    kalman_cal_pitch(pitch_raw, gyro_x);  // 卡尔曼滤波计算X倾角
    kalman_cal_roll(roll_raw, gyro_y);  // 卡尔曼滤波计算Y倾角
}

// 卡尔曼参数
static float Q_angle = 0.001;  // 角度数据置信度，角度噪声的协方差
static float Q_gyro = 0.003;  // 角速度数据置信度，角速度噪声的协方差
static float R_angle = 0.5;  // 加速度计测量噪声的协方差
static float dt = 0.01;  // 采样周期即计算任务周期10ms

void kalman_cal_pitch(float pitch, float gyro_x) // 卡尔曼滤波pitch轴计算
{
    static float Q_bias;  // Q_bias:陀螺仪的偏差
    static float K_0, K_1;  // 卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差 t_0/1:中间变量
    static float PP[2][2] = {{1, 0}, {0, 1}};  // 过程协方差矩阵P，初始值为单位阵

    pitch_kalman += (gyro_x - Q_bias) * dt;  // 状态方程,角度值等于上次最优角度加角速度减零漂后积分

    PP[0][0] = PP[0][0] + Q_angle - (PP[0][1] + PP[1][0]) * dt;
    PP[0][1] = PP[0][1] - PP[1][1] * dt;
    PP[1][0] = PP[1][0] - PP[1][1] * dt;
    PP[1][1] = PP[1][1] + Q_gyro;

    K_0 = PP[0][0] / (PP[0][0] + R_angle);
    K_1 = PP[1][0] / (PP[0][0] + R_angle);

    pitch_kalman = pitch_kalman + K_0 * (pitch - pitch_kalman);
    Q_bias = Q_bias + K_1 * (pitch - pitch_kalman);

    PP[0][0] = PP[0][0] - K_0 * PP[0][0];
    PP[0][1] = PP[0][1] - K_0 * PP[0][1];
    PP[1][0] = PP[1][0] - K_1 * PP[0][0];
    PP[1][1] = PP[1][1] - K_1 * PP[0][1];
    printf("raw: %.2f %.2f ", pitch_raw, pitch_kalman);
}

void kalman_cal_roll(float roll, float gyro_y) // 卡尔曼滤波roll轴计算
{
    static float Q_bias;  // Q_bias:陀螺仪的偏差  Angle_err:角度偏量
    static float K_0, K_1;  // 卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差 t_0/1:中间变量
    static float PP[2][2] = {{1, 0}, {0, 1}};  // 过程协方差矩阵P，初始值为单位阵

    roll_kalman += (gyro_y - Q_bias) * dt;  // 状态方程 角度值等于上次最优角度加角速度减零漂后积分
    PP[0][0] = PP[0][0] + Q_angle - (PP[0][1] + PP[1][0]) * dt;
    PP[0][1] = PP[0][1] - PP[1][1] * dt;
    PP[1][0] = PP[1][0] - PP[1][1] * dt;
    PP[1][1] = PP[1][1] + Q_gyro;

    K_0 = PP[0][0] / (PP[0][0] + R_angle);
    K_1 = PP[1][0] / (PP[0][0] + R_angle);

    roll_kalman = roll_kalman + K_0 * (roll - roll_kalman);
    Q_bias = Q_bias + K_1 * (roll - roll_kalman);

    PP[0][0] = PP[0][0] - K_0 * PP[0][0];
    PP[0][1] = PP[0][1] - K_0 * PP[0][1];
    PP[1][0] = PP[1][0] - K_1 * PP[0][0];
    PP[1][1] = PP[1][1] - K_1 * PP[0][1];
    printf("raw: %.2f %.2f\r\n", roll_raw, roll_kalman);
}
