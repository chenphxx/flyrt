#include "mpu6050.h"

#define MPU6050_ADDRESS 0xD0  // MPU6050的I2C从机地址

// 零偏校准值
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// MPU6050初始化
void mpu6050_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  // 设置为复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // 设置IO口速度为50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  // 设置为开漏输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  // 设置为上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 引脚复用
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;  // I2C模式
    I2C_InitStructure.I2C_ClockSpeed = 50000;  // 设置时钟速度为50KHz
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;  // 设置时钟占空比
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;  // 启用应答
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  // 设置7位地址
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;  // 自身地址（从机模式下有效）
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);

    mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x01);  // 电源管理寄存器1 取消睡眠模式 选择时钟源为X轴陀螺仪
    mpu6050_write_reg(MPU6050_PWR_MGMT_2, 0x00);  // 电源管理寄存器2 保持默认值0 所有轴均不待机
    mpu6050_write_reg(MPU6050_SMPLRT_DIV, 0x09);  // 采样率分频寄存器 配置采样率
    mpu6050_write_reg(MPU6050_CONFIG, 0x06);  // 配置寄存器，配置DLPF

    // 配置陀螺仪与加速度计的量程
    mpu6050_write_reg(MPU6050_GYRO_CONFIG, 0x08);  // 陀螺仪量程±500°/s
    mpu6050_write_reg(MPU6050_ACCEL_CONFIG, 0x08);  // 加速度计量程±4g
}

// 等待事件
void mpu6050_wait_event(I2C_TypeDef *i2cx, uint32_t i2c_event)
{
    uint32_t timeout;
    timeout = 10000;  // 给定超时计数时间
    while (I2C_CheckEvent(i2cx, i2c_event) != SUCCESS)  // 循环等待指定事件
    {
        timeout--;  // 等待时，计数值自减
        if (timeout == 0)  // 自减到0后，等待超时
        {
            /*超时的错误处理代码，可以添加到此处*/
            break;  // 跳出等待，不等了
        }
    }
}

// 写寄存器
void mpu6050_write_reg(uint8_t reg_address, uint8_t data)
{
    I2C_GenerateSTART(I2C1, ENABLE);  // 硬件I2C生成起始条件
    mpu6050_wait_event(I2C1, I2C_EVENT_MASTER_MODE_SELECT);  // 等待EV5

    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Transmitter);  // 硬件I2C发送从机地址，方向为发送
    mpu6050_wait_event(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);  // 等待EV6

    I2C_SendData(I2C1, reg_address);  // 硬件I2C发送寄存器地址
    mpu6050_wait_event(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING);  // 等待EV8

    I2C_SendData(I2C1, data);  // 硬件I2C发送数据
    mpu6050_wait_event(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);  // 等待EV8_2

    I2C_GenerateSTOP(I2C1, ENABLE);  // 硬件I2C生成终止条件
}

// 读寄存器
uint8_t mpu6050_read_reg(uint8_t reg_address)
{
    uint8_t data;

    I2C_GenerateSTART(I2C1, ENABLE);  // 硬件I2C生成起始条件
    mpu6050_wait_event(I2C1, I2C_EVENT_MASTER_MODE_SELECT);  // 等待EV5

    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Transmitter);  // 硬件I2C发送从机地址，方向为发送
    mpu6050_wait_event(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);  // 等待EV6

    I2C_SendData(I2C1, reg_address);  // 硬件I2C发送寄存器地址
    mpu6050_wait_event(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);  // 等待EV8_2

    I2C_GenerateSTART(I2C1, ENABLE);  // 硬件I2C生成重复起始条件
    mpu6050_wait_event(I2C1, I2C_EVENT_MASTER_MODE_SELECT);  // 等待EV5

    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Receiver);  // 硬件I2C发送从机地址，方向为接收
    mpu6050_wait_event(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);  // 等待EV6

    I2C_AcknowledgeConfig(I2C1, DISABLE);  // 在接收最后一个字节之前提前将应答失能
    I2C_GenerateSTOP(I2C1, ENABLE);  // 在接收最后一个字节之前提前申请停止条件

    mpu6050_wait_event(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED);  // 等待EV7
    data = I2C_ReceiveData(I2C1);  // 接收数据寄存器

    I2C_AcknowledgeConfig(I2C1, ENABLE);  // 将应答恢复为使能 为了不影响后续可能产生的读取多字节操作

    return data;
}

// 获取ID号
uint8_t mpu6050_get_id(void)
{
    // 返回WHO_AM_I寄存器的值
    return mpu6050_read_reg(MPU6050_WHO_AM_I);
}

// 读取原始数据数据
void mpu6050_get_odata(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
    uint8_t data_h, data_l;  // 定义数据高8位和低8位的变量

    // 加速度计X轴
    data_h = mpu6050_read_reg(MPU6050_ACCEL_XOUT_H);  // 读取加速度计X轴的高8位数据
    data_l = mpu6050_read_reg(MPU6050_ACCEL_XOUT_L);  // 读取加速度计X轴的低8位数据
    *acc_x = (data_h << 8) | data_l;  // 数据拼接，通过输出参数返回

    // 加速度计Y轴
    data_h = mpu6050_read_reg(MPU6050_ACCEL_YOUT_H);
    data_l = mpu6050_read_reg(MPU6050_ACCEL_YOUT_L);
    *acc_y = (data_h << 8) | data_l;

    // 加速度计Z轴
    data_h = mpu6050_read_reg(MPU6050_ACCEL_ZOUT_H);
    data_l = mpu6050_read_reg(MPU6050_ACCEL_ZOUT_L);
    *acc_z = (data_h << 8) | data_l;

    // 陀螺仪X轴
    data_h = mpu6050_read_reg(MPU6050_GYRO_XOUT_H);
    data_l = mpu6050_read_reg(MPU6050_GYRO_XOUT_L);
    *gyro_x = (data_h << 8) | data_l;

    // 陀螺仪Y轴
    data_h = mpu6050_read_reg(MPU6050_GYRO_YOUT_H);
    data_l = mpu6050_read_reg(MPU6050_GYRO_YOUT_L);
    *gyro_y = (data_h << 8) | data_l;

    // 陀螺仪Z轴
    data_h = mpu6050_read_reg(MPU6050_GYRO_ZOUT_H);
    data_l = mpu6050_read_reg(MPU6050_GYRO_ZOUT_L);
    *gyro_z = (data_h << 8) | data_l;
}

// 获取加速度和角速度数据
void mpu6050_get_cdata(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    // 读取原始数据
    mpu6050_get_odata(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    // 加速度计转换 量程 ±4g 8192 LSB/g
    *ax = ((float)ax_raw / 8192.0f) * 9.81f - ax_offset;
    *ay = ((float)ay_raw / 8192.0f) * 9.81f - ay_offset;
    *az = ((float)az_raw / 8192.0f) * 9.81f - az_offset;

    // 陀螺仪转换 量程 ±500°/s 65.5 LSB/(°/s)
    *gx = gx_raw / 65.5f - gx_offset;
    *gy = gy_raw / 65.5f - gy_offset;
    *gz = gz_raw / 65.5f - gz_offset;
}

// 零偏校准器
void mpu6050_calibrate(void)
{
    float ax, ay, az, gx, gy, gz;  // 未校准的加速度和角速度

    // 临时偏移量
    float ax_offset_temp = 0, ay_offset_temp = 0, az_offset_temp = 0;
    float gx_offset_temp = 0, gy_offset_temp = 0, gz_offset_temp = 0;

    // 零偏校准值
    ax_offset = 0, ay_offset = 0, az_offset = 0;
    gx_offset = 0, gy_offset = 0, gz_offset = 0;

    for (int i = 0; i < 100; i++)
    {
        mpu6050_get_cdata(&ax, &ay, &az, &gx, &gy, &gz);  // 获取加速度和角速度数据
        ax_offset_temp += ax;  // 累加
        ay_offset_temp += ay;
        // az_offset_temp += az;  // 由于重力加速度的存在 Z轴不做校准
        gx_offset_temp += gx;
        gy_offset_temp += gy;
        gz_offset_temp += gz;
    }

    // // 计算偏移量
    ax_offset = ax_offset_temp / 100;
    ay_offset = ay_offset_temp / 100;
    // az_offset = az_offset_temp / 100;  // 由于重力加速度的存在 Z轴不做校准
    gx_offset = gx_offset_temp / 100;
    gy_offset = gy_offset_temp / 100;
    gz_offset = gz_offset_temp / 100;
}
