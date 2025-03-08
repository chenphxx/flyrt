#include "mpu6050.h"

#define MPU6050_ADDRESS 0xD0  // MPU6050的I2C从机地址

// MPU6050初始化
void MPU6050_Init(void)
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

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);  // PB6为I2C1_SCL
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);  // PB7为I2C1_SDA

    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;  // I2C模式
    I2C_InitStructure.I2C_ClockSpeed = 50000;  // 设置时钟速度为50KHz
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;  // 设置时钟占空比
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;  // 启用应答
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  // 设置7位地址
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;  // 自身地址（从机模式下有效）
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);  // 电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);  // 电源管理寄存器2，保持默认值0，所有轴均不待机
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);  // 采样率分频寄存器，配置采样率
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);  // 配置寄存器，配置DLPF
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);  // 陀螺仪配置寄存器，选择满量程为±2000°/s
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);  // 加速度计配置寄存器，选择满量程为±16g
}

// 等待事件
void MPU6050_WaitEvent(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT)
{
    uint32_t Timeout;
    Timeout = 10000;  // 给定超时计数时间
    while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)  // 循环等待指定事件
    {
        Timeout--;  // 等待时，计数值自减
        if (Timeout == 0)  // 自减到0后，等待超时
        {
            /*超时的错误处理代码，可以添加到此处*/
            break;  // 跳出等待，不等了
        }
    }
}

// 写寄存器
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    I2C_GenerateSTART(I2C1, ENABLE);  // 硬件I2C生成起始条件
    MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);  // 等待EV5

    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Transmitter);  // 硬件I2C发送从机地址，方向为发送
    MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);  // 等待EV6

    I2C_SendData(I2C1, RegAddress);  // 硬件I2C发送寄存器地址
    MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING);  // 等待EV8

    I2C_SendData(I2C1, Data);  // 硬件I2C发送数据
    MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);  // 等待EV8_2

    I2C_GenerateSTOP(I2C1, ENABLE);  // 硬件I2C生成终止条件
}

// 读寄存器
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    I2C_GenerateSTART(I2C1, ENABLE);  // 硬件I2C生成起始条件
    MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);  // 等待EV5

    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Transmitter);  // 硬件I2C发送从机地址，方向为发送
    MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);  // 等待EV6

    I2C_SendData(I2C1, RegAddress);  // 硬件I2C发送寄存器地址
    MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);  // 等待EV8_2

    I2C_GenerateSTART(I2C1, ENABLE);  // 硬件I2C生成重复起始条件
    MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);  // 等待EV5

    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Receiver);  // 硬件I2C发送从机地址，方向为接收
    MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);  // 等待EV6

    I2C_AcknowledgeConfig(I2C1, DISABLE);  // 在接收最后一个字节之前提前将应答失能
    I2C_GenerateSTOP(I2C1, ENABLE);  // 在接收最后一个字节之前提前申请停止条件

    MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED);  // 等待EV7
    Data = I2C_ReceiveData(I2C1);  // 接收数据寄存器

    I2C_AcknowledgeConfig(I2C1, ENABLE);  // 将应答恢复为使能，为了不影响后续可能产生的读取多字节操作

    return Data;
}

// 获取ID号
uint8_t MPU6050_GetID(void)
{
    // 返回WHO_AM_I寄存器的值
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

// 读取数据
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t DataH, DataL;  // 定义数据高8位和低8位的变量

    // 加速度计X轴
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);  // 读取加速度计X轴的高8位数据
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);  // 读取加速度计X轴的低8位数据
    *AccX = (DataH << 8) | DataL;  // 数据拼接，通过输出参数返回

    // 加速度计Y轴
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    *AccY = (DataH << 8) | DataL;

    // 加速度计Z轴
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    *AccZ = (DataH << 8) | DataL;

    // 陀螺仪X轴
    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    *GyroX = (DataH << 8) | DataL;

    // 陀螺仪Y轴
    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    *GyroY = (DataH << 8) | DataL;

    // 陀螺仪Z轴
    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    *GyroZ = (DataH << 8) | DataL;
}
