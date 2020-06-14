#ifndef _soft_iic_H
#define _soft_iic_H

typedef unsigned char u8;

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(u8 ack);			//IIC读取一个字节
u8 I2C_WRITE_STRING(u8 address, u8* p_data, u8 length);
u8 I2C_READ_STRING(u8 address, u8* p_data, u8 length);

#endif
