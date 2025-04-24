#include "iic.h"
#include "stdlib.h"

I2C_BUS *Pthis = 0; // 全局指针

typedef struct I2C_Private
{
    GPIO_TypeDef *GPIOx;
    I2C_TypeDef *I2Cx;
    uint16_t SCL;
    uint16_t SDA;
    uint16_t I2C_Add;
    uint8_t Hard_I2C_EN;
} I2C_Private;

//PB6->SCL
#define RCC_IIC_SCL 	RCC_AHB1Periph_GPIOB	//端口时钟
#define IIC_SCL_PORT	GPIOB					//端口
#define IIC_SCL			GPIO_Pin_6				//引脚

//PB7->SDA
#define RCC_IIC_SDA		RCC_AHB1Periph_GPIOB	//端口时钟
#define IIC_SDA_PORT	GPIOB					//端口
#define IIC_SDA			GPIO_Pin_7				//引脚

//IO操作
#define IIC_SCL_H		GPIO_SetBits(IIC_SCL_PORT, IIC_SCL)
#define IIC_SCL_L		GPIO_ResetBits(IIC_SCL_PORT, IIC_SCL)
#define IIC_SDA_H		GPIO_SetBits(IIC_SDA_PORT, IIC_SDA)
#define IIC_SDA_L		GPIO_ResetBits(IIC_SDA_PORT, IIC_SDA)
#define READ_SDA		(IIC_SDA_PORT->IDR&IIC_SDA) ? 1 : 0	//输入SDA

void MyI2C_W_SCL(uint8_t BitValue)
{
    if (BitValue)
        IIC_SCL_H;
    else
        IIC_SCL_L;
    delay_us(SI2C_delay_time);
}

void MyI2C_W_SDA(uint8_t BitValue)
{
    if (BitValue)
        IIC_SDA_H;
    else
        IIC_SDA_L;
    delay_us(SI2C_delay_time);
}

uint8_t MyI2C_R_SDA()
{
    uint8_t BitValue = GPIO_ReadInputDataBit(Pthis->Private->GPIOx, Pthis->Private->SDA);
    delay_us(SI2C_delay_time);
    return BitValue;
}

void SI2C_Start()
{
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(0);
}

void SI2C_Stop()
{
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(1);
}

void SI2C_WriteByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        MyI2C_W_SDA(Byte & (0x80 >> i));
        delay_us(SI2C_delay_time);
        MyI2C_W_SCL(1);
        delay_us(SI2C_delay_time);
        MyI2C_W_SCL(0);
        delay_us(SI2C_delay_time);
    }
}

uint8_t SI2C_ReceiveByte()
{
    uint8_t i, Byte = 0x00;
    MyI2C_W_SDA(1);
    for (i = 0; i < 8; i++)
    {
        MyI2C_W_SCL(1);
        if (MyI2C_R_SDA() == 1)
        {
            Byte |= (0x80 >> i);
        }
        MyI2C_W_SCL(0);
    }
    return Byte;
}

void SI2C_WriteAck(uint8_t AckBit)
{
    MyI2C_W_SDA(AckBit);
    MyI2C_W_SCL(1);
    MyI2C_W_SCL(0);
}

uint8_t SI2C_ReceiveAck()
{
    uint8_t AckBit;
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    AckBit = MyI2C_R_SDA();
    MyI2C_W_SCL(0);
    return AckBit;
}

uint8_t SI2C_ACK_Test()
{
    uint8_t Ack;
    SI2C_Start();
    SI2C_WriteByte(Pthis->Private->I2C_Add);
    Ack = SI2C_ReceiveAck();
    SI2C_Stop();
    return Ack;
}

void HI2C_WaitEvent(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT)
{
    uint32_t Timeout;
    Timeout = 1000;
    while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
    {
        Timeout--; 
        if (Timeout == 0)
            break;
    }
}

void I2C_Write_Reg(uint8_t RegAddress, uint8_t Data)
{
    if (Pthis->Private->Hard_I2C_EN)
    {
        I2C_GenerateSTART(Pthis->Private->I2Cx, ENABLE);
        HI2C_WaitEvent(Pthis->Private->I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

        I2C_Send7bitAddress(Pthis->Private->I2Cx, Pthis->Private->I2C_Add, I2C_Direction_Transmitter);
        HI2C_WaitEvent(Pthis->Private->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

        I2C_SendData(Pthis->Private->I2Cx, RegAddress);
        HI2C_WaitEvent(Pthis->Private->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING);

        I2C_SendData(Pthis->Private->I2Cx, Data);
        HI2C_WaitEvent(Pthis->Private->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

        I2C_GenerateSTOP(Pthis->Private->I2Cx, ENABLE);
    }
    else
    {
        SI2C_Start();

        SI2C_WriteByte(Pthis->Private->I2C_Add);
        SI2C_ReceiveAck();

        SI2C_WriteByte(RegAddress);
        SI2C_ReceiveAck();

        SI2C_WriteByte(Data);
        SI2C_ReceiveAck();

        SI2C_Stop();
    }
}

uint8_t I2C_Read_Reg(uint8_t RegAddress)
{
    uint8_t Data;
    if (Pthis->Private->Hard_I2C_EN)
    {
        I2C_GenerateSTART(Pthis->Private->I2Cx, ENABLE);
        HI2C_WaitEvent(Pthis->Private->I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

        I2C_Send7bitAddress(Pthis->Private->I2Cx, Pthis->Private->I2C_Add, I2C_Direction_Transmitter);
        HI2C_WaitEvent(Pthis->Private->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

        I2C_SendData(Pthis->Private->I2Cx, RegAddress);
        HI2C_WaitEvent(Pthis->Private->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

        I2C_GenerateSTART(Pthis->Private->I2Cx, ENABLE);
        HI2C_WaitEvent(Pthis->Private->I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

        I2C_Send7bitAddress(Pthis->Private->I2Cx, Pthis->Private->I2C_Add, I2C_Direction_Receiver);
        HI2C_WaitEvent(Pthis->Private->I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

        I2C_AcknowledgeConfig(Pthis->Private->I2Cx, DISABLE);
        I2C_GenerateSTOP(Pthis->Private->I2Cx, ENABLE);

        HI2C_WaitEvent(Pthis->Private->I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED);
        Data = I2C_ReceiveData(Pthis->Private->I2Cx);

        I2C_AcknowledgeConfig(Pthis->Private->I2Cx, ENABLE);

        return Data;
    }
    else
    {
        SI2C_Start();
        SI2C_WriteByte(Pthis->Private->I2C_Add);
        SI2C_ReceiveAck();
        SI2C_WriteByte(RegAddress);
        SI2C_ReceiveAck();

        SI2C_Start();
        SI2C_WriteByte(Pthis->Private->I2C_Add | 0x01);
        SI2C_ReceiveAck();
        Data = SI2C_ReceiveByte();
        SI2C_WriteAck(1);
        SI2C_Stop();

        return Data;
    }
}

// 软件IIC
I2C_BUS Create_SI2C(GPIO_TypeDef *GPIOx, uint16_t SCL, uint16_t SDA, uint8_t Address)
{
    struct I2C_BUS this;
    this.Private = malloc(sizeof(I2C_Private));
    this.Private->I2C_Add = Address;
    this.Private->GPIOx = GPIOx;
    this.Private->SCL = SCL;
    this.Private->SDA = SDA;
    this.Private->I2Cx = 0;
    this.Private->Hard_I2C_EN = 0;

    this.AckTest = SI2C_ACK_Test;
    this.Write_Reg = I2C_Write_Reg;
    this.Read_Reg = I2C_Read_Reg;
    this.Rest_Speed = 0;

    this.Read_Reg_continue = 0;
    this.Write_Reg_continue = 0;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_Init_Struct;

    GPIO_Init_Struct.GPIO_Pin = SCL | SDA;
    GPIO_Init_Struct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init_Struct.GPIO_OType = GPIO_OType_OD;
    GPIO_Init_Struct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_Init_Struct);

    GPIO_WriteBit(GPIOx, SCL, (BitAction)1);
    GPIO_WriteBit(GPIOx, SDA, (BitAction)1);

    return this;
}
