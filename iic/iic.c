#include "iic.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"


#define IIC_OK		(0)
#define IIC_FAIL	(1)
//IO��������	 
#define IIC_SCL(x)    	nrf_gpio_pin_write(ARDUINO_SCL_PIN, x) 	//SCL
#define IIC_SDA(x)    	nrf_gpio_pin_write(ARDUINO_SDA_PIN, x) 	//SDA	 
#define READ_SDA		nrf_gpio_pin_read(ARDUINO_SDA_PIN)  	//����SDA

static void delay_us(uint32_t us_time)
{
	nrf_delay_us(us_time);
}

/*******************************************************************************
* �� �� ��         : IIC_Init
* ��������		   : IIC��ʼ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_Init(void)
{
	nrf_gpio_cfg_output(ARDUINO_SCL_PIN);
	nrf_gpio_cfg_output(ARDUINO_SDA_PIN);
	nrf_gpio_pin_write(ARDUINO_SCL_PIN, 1);
	nrf_gpio_pin_write(ARDUINO_SDA_PIN, 1);
	
	IIC_Start();
	delay_us(2);
	IIC_Stop();
	delay_us(2);
}

/*******************************************************************************
* �� �� ��         : SDA_OUT
* ��������		   : SDA�������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void SDA_OUT(void)
{
	nrf_gpio_cfg_output(ARDUINO_SDA_PIN);
}

/*******************************************************************************
* �� �� ��         : SDA_IN
* ��������		   : SDA��������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void SDA_IN(void)
{
	nrf_gpio_cfg_input(ARDUINO_SDA_PIN, NRF_GPIO_PIN_PULLUP);
}

/*******************************************************************************
* �� �� ��         : IIC_Start
* ��������		   : ����IIC��ʼ�ź�   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA(1);	  	  
	IIC_SCL(1);
	delay_us(2);
	IIC_SDA(0);	//START:when CLK is high,DATA change form high to low 
	delay_us(2);
	IIC_SCL(0);//ǯסI2C���ߣ�׼�����ͻ�������� 
}	

/*******************************************************************************
* �� �� ��         : IIC_Stop
* ��������		   : ����IICֹͣ�ź�   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL(0);
	IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
 	IIC_SCL(1); 
	delay_us(2); 
	IIC_SDA(1);//����I2C���߽����ź�
	delay_us(2);							   	
}

/*******************************************************************************
* �� �� ��         : IIC_Wait_Ack
* ��������		   : �ȴ�Ӧ���źŵ���   
* ��    ��         : ��
* ��    ��         : 1������Ӧ��ʧ��
        			 0������Ӧ��ɹ�
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 tempTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA(1);
	delay_us(1);	   
	IIC_SCL(1);
	delay_us(1);	 
	while(READ_SDA)
	{
		tempTime++;
		delay_us(1);
		if(tempTime>5)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL(0);//ʱ�����0 	   
	return 0;  
} 

/*******************************************************************************
* �� �� ��         : IIC_Ack
* ��������		   : ����ACKӦ��  
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(0);
	delay_us(2);
	IIC_SCL(1);
	delay_us(2);
	IIC_SCL(0);
}

/*******************************************************************************
* �� �� ��         : IIC_NAck
* ��������		   : ����NACK��Ӧ��  
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/		    
void IIC_NAck(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(1);
	delay_us(2);
	IIC_SCL(1);
	delay_us(2);
	IIC_SCL(0);
}	

/*******************************************************************************
* �� �� ��         : IIC_Send_Byte
* ��������		   : IIC����һ���ֽ� 
* ��    ��         : txd������һ���ֽ�
* ��    ��         : ��
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL(0);//����ʱ�ӿ�ʼ���ݴ���
	delay_us(2);
    for(t=0;t<8;t++)
    {              
        if((txd&0x80)>0) //0x80  1000 0000
			IIC_SDA(1);
		else
			IIC_SDA(0);
        txd<<=1; 	  
		delay_us(2);
		IIC_SCL(1);
		delay_us(2); 
		IIC_SCL(0);	
		delay_us(2);
    }	 
} 

/*******************************************************************************
* �� �� ��         : IIC_Read_Byte
* ��������		   : IIC��һ���ֽ� 
* ��    ��         : ack=1ʱ������ACK��ack=0������nACK 
* ��    ��         : Ӧ����Ӧ��
*******************************************************************************/  
u8 IIC_Read_Byte(u8 ack)
{
	u8 i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL(0); 
        delay_us(2);
		IIC_SCL(1);
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(2); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

u8 I2C_WRITE_STRING(u8 address, u8* p_data, u8 length)
{
	u8 nCount = 0;
	u8 ret = IIC_OK;
	
	IIC_Start();
	
	IIC_Send_Byte(~0x01 & (address << 1));
	ret = IIC_Wait_Ack();
	
	for(; nCount < length ; ++nCount)
	{
		IIC_Send_Byte(p_data[nCount]);
		ret += IIC_Wait_Ack();
	}
	
	IIC_Stop();
	
	return ret;
}

u8 I2C_READ_STRING(u8 address, u8* p_data, u8 length)
{
	u8 nCount = 0;
	u8 ret = IIC_OK;
	
	IIC_Start();
	
	IIC_Send_Byte(~0x01 & (address << 1));
	ret = IIC_Wait_Ack();
	
	IIC_Send_Byte(p_data[0]);
	ret += IIC_Wait_Ack();
	
	IIC_Start();
	
	IIC_Send_Byte(0x01 | (address << 1));
	ret += IIC_Wait_Ack();
	
	for(nCount = 0 ; nCount < length ; ++nCount)
	{
		if(nCount == (length - 1) )
			p_data[nCount] = IIC_Read_Byte(0);
		else
			p_data[nCount] = IIC_Read_Byte(1);
	}	
	
	IIC_Stop();
	
	return ret;
}




