#include "mpu6050.h"

I2C_BUS MPU6050_I2C;

typedef enum
{
    Band_256Hz = 0x00,
    Band_186Hz,
    Band_96Hz,
    Band_43Hz,
    Band_21Hz,
    Band_10Hz,
    Band_5Hz
} Filter_Typedef;

typedef enum
{
    gyro_250 = 0x00,
    gyro_500 = 0x08,
    gyro_1000 = 0x10,
    gyro_2000 = 0x18
} GYRO_CONFIG_Typedef;

typedef enum
{
    acc_2g = 0x00,
    acc_4g = 0x08,
    acc_8g = 0x10,
    acc_16g = 0x18
} ACCEL_CONFIG_Typedef;

typedef enum
{
    FIFO_Disable,
    Acc_OUT = 0x08,
    Gyro_zOUT = 0x10,
    Gyro_yOUT = 0x20,
    Gyro_xOUT = 0x40,
    Temp_OUT = 0x80,
} FIFO_EN_Typedef;

typedef enum
{
    interrupt_Disable,
    Data_Ready_EN = 0x01,
    I2C_Master_EN = 0x08,
    FIFO_overFolow_EN = 0x10,
    Motion_EN = 0x40,
} INT_EN_Typedef;

typedef struct MPU6050_InitTypeDef
{
    uint16_t SMPLRT_Rate;
    Filter_Typedef Filter;
    GYRO_CONFIG_Typedef gyro_range;
    ACCEL_CONFIG_Typedef acc_range;
    FIFO_EN_Typedef FIFO_EN;
    INT_EN_Typedef INT;
} MPU6050_InitTypeDef;

static void mpu6050_register_init(MPU6050_InitTypeDef *this)
{
    I2C(MPU6050_I2C)->Write_Reg(MPU6050_PWR_MGMT_1, 0x80);  // 复位

    delay_ms(100);
    I2C(MPU6050_I2C)->Write_Reg(MPU6050_PWR_MGMT_1, 0x00);  // 唤醒
    uint8_t SMPLRT_DIV;
    if (this->SMPLRT_Rate > 1000)
        this->SMPLRT_Rate = 1000;
    else if (this->SMPLRT_Rate < 4)
        this->SMPLRT_Rate = 4;
    SMPLRT_DIV = 1000 / this->SMPLRT_Rate - 1;  // 由计算公式得

    I2C(MPU6050_I2C)->Write_Reg(MPU6050_SMPLRT_DIV, SMPLRT_DIV);
    I2C(MPU6050_I2C)->Write_Reg(MPU6050_INT_ENABLE, this->INT);
    I2C(MPU6050_I2C)->Write_Reg(MPU6050_CONFIG, this->Filter);
    I2C(MPU6050_I2C)->Write_Reg(MPU6050_GYRO_CONFIG, this->gyro_range);
    I2C(MPU6050_I2C)->Write_Reg(MPU6050_ACCEL_CONFIG, this->acc_range);
    I2C(MPU6050_I2C)->Write_Reg(MPU6050_FIFO_EN, this->FIFO_EN);

    uint8_t temp = 0x00;
    if (this->FIFO_EN != 0x00)  // 如果打开了FIFO
        temp = 0x40;
    if ((this->INT & 0x08) == 1)  // 如果打开了中断
        temp |= 0x08;
    I2C(MPU6050_I2C)->Write_Reg(MPU6050_USER_CTRL, temp);
    I2C(MPU6050_I2C)->Write_Reg(MPU6050_PWR_MGMT_1, 0x01);  // X轴为参考
}

uint8_t mpu6050_id(void)
{
    return I2C(MPU6050_I2C)->Read_Reg(MPU6050_WHO_AM_I);
}

void mpu6050_init(GPIO_TypeDef *GPIOx, uint16_t SCl, uint16_t SDA)
{
    MPU6050_I2C = Create_SI2C(GPIOx, SCl, SDA, MPU6050_ADDRESS << 1);

    MPU6050_InitTypeDef MPU6050_init_Struct;
    MPU6050_init_Struct.SMPLRT_Rate = 500;  // 采样率Hz
    MPU6050_init_Struct.Filter = Band_5Hz;  // 低通滤波器带宽
    MPU6050_init_Struct.gyro_range = gyro_2000;  // 陀螺仪测量范围
    MPU6050_init_Struct.acc_range = acc_16g;  // 加速度计测量范围
    MPU6050_init_Struct.FIFO_EN = FIFO_Disable;  // FIFO
    MPU6050_init_Struct.INT = interrupt_Disable;  // 中断配置

    mpu6050_register_init(&MPU6050_init_Struct); // 初始化寄存器
}

void mpu6050_get_raw(MPU6050_Raw *this)
{
    this->AccX = ((int16_t)(I2C(MPU6050_I2C)->Read_Reg(MPU6050_ACCEL_XOUT_H)) << 8) | I2C(MPU6050_I2C)->Read_Reg(MPU6050_ACCEL_XOUT_L);
    this->AccY = ((int16_t)(I2C(MPU6050_I2C)->Read_Reg(MPU6050_ACCEL_YOUT_H)) << 8) | I2C(MPU6050_I2C)->Read_Reg(MPU6050_ACCEL_YOUT_L);
    this->AccZ = ((int16_t)(I2C(MPU6050_I2C)->Read_Reg(MPU6050_ACCEL_ZOUT_H)) << 8) | I2C(MPU6050_I2C)->Read_Reg(MPU6050_ACCEL_ZOUT_L);
    this->GyroX = ((int16_t)(I2C(MPU6050_I2C)->Read_Reg(MPU6050_GYRO_XOUT_H)) << 8) | I2C(MPU6050_I2C)->Read_Reg(MPU6050_GYRO_XOUT_L);
    this->GyroX = ((int16_t)(I2C(MPU6050_I2C)->Read_Reg(MPU6050_GYRO_YOUT_H)) << 8) | I2C(MPU6050_I2C)->Read_Reg(MPU6050_GYRO_YOUT_L);
    this->GyroX = ((int16_t)(I2C(MPU6050_I2C)->Read_Reg(MPU6050_GYRO_ZOUT_H)) << 8) | I2C(MPU6050_I2C)->Read_Reg(MPU6050_GYRO_ZOUT_L);
    this->Temp = ((uint16_t)(I2C(MPU6050_I2C)->Read_Reg(MPU6050_TEMP_OUT_H)) << 8) | I2C(MPU6050_I2C)->Read_Reg(MPU6050_TEMP_OUT_L);
}

float roll_offset = 0, pitch_offset = 0;

void mpu6050_get_angle(MPU6050 *this)
{
    int16_t temp = 0;
    float Ax, Ay, Az = 0;
    float Gx, Gy, Gz = 0;

    static float Gyroscope_roll = 0;  // 陀螺仪积分roll
    static float Gyroscope_pitch = 0;  // 陀螺仪积分pitch
    const static float dt = 0.005;  // 采样间隔
    const static float weight = 0.95;  // 互补滤波权重

    temp = ((uint16_t)I2C(MPU6050_I2C)->Read_Reg(MPU6050_ACCEL_XOUT_H) << 8) + I2C(MPU6050_I2C)->Read_Reg(MPU6050_ACCEL_XOUT_L);
    Ax = temp * 16.0 / 32768;

    temp = ((uint16_t)I2C(MPU6050_I2C)->Read_Reg(MPU6050_ACCEL_YOUT_H) << 8) + I2C(MPU6050_I2C)->Read_Reg(MPU6050_ACCEL_YOUT_L);
    Ay = temp * 16.0 / 32768;

    temp = ((uint16_t)I2C(MPU6050_I2C)->Read_Reg(MPU6050_ACCEL_ZOUT_H) << 8) + I2C(MPU6050_I2C)->Read_Reg(MPU6050_ACCEL_ZOUT_L);
    Az = temp * 16.0 / 32768;

    temp = ((uint16_t)I2C(MPU6050_I2C)->Read_Reg(MPU6050_GYRO_XOUT_H) << 8) + I2C(MPU6050_I2C)->Read_Reg(MPU6050_GYRO_XOUT_L);
    Gx = temp * dt * 0.0174533;

    temp = ((uint16_t)I2C(MPU6050_I2C)->Read_Reg(MPU6050_GYRO_YOUT_H) << 8) + I2C(MPU6050_I2C)->Read_Reg(MPU6050_GYRO_YOUT_L);
    Gy = temp * dt * 0.0174533;

    temp = ((uint16_t)I2C(MPU6050_I2C)->Read_Reg(MPU6050_GYRO_ZOUT_H) << 8) + I2C(MPU6050_I2C)->Read_Reg(MPU6050_GYRO_ZOUT_L);
    Gz = temp * dt * 0.0174533;

    // 互补滤波
    Gyroscope_roll += Gy;
    Gyroscope_pitch += Gx;
    // this->roll = weight * atan2(Ay, Az) / 3.1415926 * 180 + (1 - weight) * Gyroscope_roll - roll_offset;
    // this->pitch = -(weight * atan2(Ax, Az) / 3.1415926 * 180 + (1 - weight) * Gyroscope_pitch) - pitch_offset;
    // this->yaw += Gz * 20 + 0.0157;  // 减小零飘

    // 仅使用加速度计数据计算角度数据 不使用互补滤波 互补滤波会导致角度数据随着时间漂移
    this->roll = atan2(Ay, Az) / 3.1415926 * 180 - roll_offset;
    this->pitch = -(atan2(Ax, Az) / 3.1415926 * 180) - pitch_offset;
}

// 校准角度 添加偏移量
void mpu6050_calibrate(void)
{
    // 临时偏移量
    float roll_offset_temp = 0, pitch_offset_temp = 0;

    MPU6050 temp;

    for (int i = 0; i < 200; i++)
    {
        mpu6050_get_angle(&temp);
        roll_offset_temp += temp.roll;
        pitch_offset_temp += temp.pitch;
        // delay_ms(1);
    }
    roll_offset = roll_offset_temp / 200;
    pitch_offset = pitch_offset_temp / 200;
}

// 获取转换后的温度数据
float mpu6050_get_temp(void)
{
    uint8_t temp = (((uint16_t)I2C(MPU6050_I2C)->Read_Reg(MPU6050_TEMP_OUT_H)) << 8) | I2C(MPU6050_I2C)->Read_Reg(MPU6050_TEMP_OUT_L);
    float temperature = (float)temp / 340 + 36.53;

    return temperature;
}
