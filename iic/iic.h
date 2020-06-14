#ifndef _soft_iic_H
#define _soft_iic_H

typedef unsigned char u8;

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(u8 ack);			//IIC��ȡһ���ֽ�
u8 I2C_WRITE_STRING(u8 address, u8* p_data, u8 length);
u8 I2C_READ_STRING(u8 address, u8* p_data, u8 length);

#endif
