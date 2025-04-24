#include "filter.h"

float accel_x;  // X轴加速度值
float accel_y;  // Y轴加速度值
float accel_z;  // Z轴加速度值
float gyro_x;  // X轴陀螺仪角速度数据
float gyro_y;  // Y轴陀螺仪角速度数据
float gyro_z;  // Z轴陀螺仪角速度数据

float ekf_yaw_raw;  // 航线角yaw原始数据
float ekf_yaw_kalman;  // yaw滤波后数据

float ekf_pitch_raw;  // pitch原始数据
float ekf_pitch_kalman;  // pitch滤波后数据
float ekf_roll_raw;  // roll原始数据
float ekf_roll_kalman;  // roll滤波后数据

short temp;  // 温度

float ekf_roll_offset = 0;  // roll偏移量
float ekf_pitch_offset = 0;  // pitch偏移量

float ekf_roll_cor, ekf_pitch_cor;  // 去除偏移量的值

// 卡尔曼参数
static float Q_angle = 0.003;  // 角度数据置信度，角度噪声的协方差
static float Q_gyro = 0.003;  // 角速度数据置信度，角速度噪声的协方差
static float R_angle = 0.5;  // 加速度计测量噪声的协方差
static float dt = 0.01;  // 采样周期即计算任务周期10ms

void ekf_pitch(float pitch, float gyro_x) // 卡尔曼滤波pitch轴计算
{
    static float Q_bias;  // Q_bias:陀螺仪的偏差
    static float K_0, K_1;  // 卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差 t_0/1:中间变量
    static float PP[2][2] = {{1, 0}, {0, 1}};  // 过程协方差矩阵P，初始值为单位阵

    ekf_pitch_kalman += (gyro_x - Q_bias) * dt;  // 状态方程,角度值等于上次最优角度加角速度减零漂后积分

    PP[0][0] = PP[0][0] + Q_angle - (PP[0][1] + PP[1][0]) * dt;
    PP[0][1] = PP[0][1] - PP[1][1] * dt;
    PP[1][0] = PP[1][0] - PP[1][1] * dt;
    PP[1][1] = PP[1][1] + Q_gyro;

    K_0 = PP[0][0] / (PP[0][0] + R_angle);
    K_1 = PP[1][0] / (PP[0][0] + R_angle);

    ekf_pitch_kalman = ekf_pitch_kalman + K_0 * (pitch - ekf_pitch_kalman);
    Q_bias = Q_bias + K_1 * (pitch - ekf_pitch_kalman);

    PP[0][0] = PP[0][0] - K_0 * PP[0][0];
    PP[0][1] = PP[0][1] - K_0 * PP[0][1];
    PP[1][0] = PP[1][0] - K_1 * PP[0][0];
    PP[1][1] = PP[1][1] - K_1 * PP[0][1];

    // printf("未滤波pitch: %.2f 扩展滤波pitch: %.2f ", ekf_pitch_raw, ekf_pitch_kalman);
}

void ekf_roll(float roll, float gyro_y) // 卡尔曼滤波roll轴计算
{
    static float Q_bias;  // Q_bias:陀螺仪的偏差  Angle_err:角度偏量
    static float K_0, K_1;  // 卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差 t_0/1:中间变量
    static float PP[2][2] = {{1, 0}, {0, 1}};  // 过程协方差矩阵P，初始值为单位阵

    ekf_roll_kalman += (gyro_y - Q_bias) * dt;  // 状态方程 角度值等于上次最优角度加角速度减零漂后积分
    PP[0][0] = PP[0][0] + Q_angle - (PP[0][1] + PP[1][0]) * dt;
    PP[0][1] = PP[0][1] - PP[1][1] * dt;
    PP[1][0] = PP[1][0] - PP[1][1] * dt;
    PP[1][1] = PP[1][1] + Q_gyro;

    K_0 = PP[0][0] / (PP[0][0] + R_angle);
    K_1 = PP[1][0] / (PP[0][0] + R_angle);

    ekf_roll_kalman = ekf_roll_kalman + K_0 * (roll - ekf_roll_kalman) ;
    Q_bias = Q_bias + K_1 * (roll - ekf_roll_kalman);

    PP[0][0] = PP[0][0] - K_0 * PP[0][0];
    PP[0][1] = PP[0][1] - K_0 * PP[0][1];
    PP[1][0] = PP[1][0] - K_1 * PP[0][0];
    PP[1][1] = PP[1][1] - K_1 * PP[0][1];

    // printf("未滤波roll: %.2f 扩展滤波roll: %.2f\r\n", ekf_roll_raw, ekf_roll_kalman);
}

// 执行扩展卡尔曼滤波
void ekf_exec(void)
{
    // 原始数据获取
    float accx, accy, accz;  // 三方向角加速度值
	MPU6050_Raw ret;
    
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
        ekf_pitch_raw = 1;
        ekf_roll_raw = 1;
    }
    else
    {
        ekf_pitch_raw = (atan(accy / accz)) * 180 / 3.14;
        ekf_roll_raw = (atan(accx / accz)) * 180 / 3.14;
    }

    // 判断计算后角度的正负号
    if (accel_x < 32764)
        ekf_roll_raw = +ekf_roll_raw;
    if (accel_x > 32764)
        ekf_roll_raw = -ekf_roll_raw;
    if (accel_y < 32764)
        ekf_pitch_raw = +ekf_pitch_raw;
    if (accel_y > 32764)
        ekf_pitch_raw = -ekf_pitch_raw;

    // 角速度原始值处理过程
    // 陀螺仪配置寄存器0X1B内写入0x18，设置范围为±2000deg/s。换算关系：2^16/4000=16.4LSB/(°/S)
    // 计算角速度
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

    ekf_pitch(ekf_pitch_raw, gyro_x);  // 卡尔曼滤波计算X倾角
    ekf_roll(ekf_roll_raw, gyro_y);  // 卡尔曼滤波计算Y倾角

    // 根据需求的不同 可以选择使用未滤波数据减去偏移量还是使用滤波后的数据减去偏移量 这里为了保障数据的响应 选择未滤波数据
    // ekf_roll_cor = ekf_roll_raw - ekf_roll_offset;
    // ekf_pitch_cor = ekf_pitch_raw - ekf_pitch_offset;

    ekf_roll_cor = ekf_roll_kalman - ekf_roll_offset;
    ekf_pitch_cor = ekf_pitch_kalman - ekf_pitch_offset;
    printf("%.2f, %.2f\r\n", ekf_roll_cor, ekf_pitch_cor);
}

// 校准初始角度
void ekf_calibrate(void)
{
    float roll_offset_temp = 0, pitch_offset_temp = 0;

    for (int i = 0; i < 200; i++)
    {
        ekf_exec();
        roll_offset_temp += ekf_roll_kalman;
        pitch_offset_temp += ekf_pitch_kalman;
    }
    ekf_roll_offset = roll_offset_temp / 200;
    ekf_pitch_offset = pitch_offset_temp / 200;

    // 必须加这一行printf 否则会导致编译器负优化 得出错误的校准值 还未找到原因 等待后续重构
    printf("偏移量: %.2f %.2f\r\n", ekf_roll_offset, ekf_pitch_offset);
}
