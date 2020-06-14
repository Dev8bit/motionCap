#include "iic.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"


#define IIC_OK		(0)
#define IIC_FAIL	(1)
//IO操作函数	 
#define IIC_SCL(x)    	nrf_gpio_pin_write(ARDUINO_SCL_PIN, x) 	//SCL
#define IIC_SDA(x)    	nrf_gpio_pin_write(ARDUINO_SDA_PIN, x) 	//SDA	 
#define READ_SDA		nrf_gpio_pin_read(ARDUINO_SDA_PIN)  	//输入SDA

static void delay_us(uint32_t us_time)
{
	nrf_delay_us(us_time);
}

/*******************************************************************************
* 函 数 名         : IIC_Init
* 函数功能		   : IIC初始化
* 输    入         : 无
* 输    出         : 无
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
* 函 数 名         : SDA_OUT
* 函数功能		   : SDA输出配置	   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void SDA_OUT(void)
{
	nrf_gpio_cfg_output(ARDUINO_SDA_PIN);
}

/*******************************************************************************
* 函 数 名         : SDA_IN
* 函数功能		   : SDA输入配置	   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void SDA_IN(void)
{
	nrf_gpio_cfg_input(ARDUINO_SDA_PIN, NRF_GPIO_PIN_PULLUP);
}

/*******************************************************************************
* 函 数 名         : IIC_Start
* 函数功能		   : 产生IIC起始信号   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA(1);	  	  
	IIC_SCL(1);
	delay_us(2);
	IIC_SDA(0);	//START:when CLK is high,DATA change form high to low 
	delay_us(2);
	IIC_SCL(0);//钳住I2C总线，准备发送或接收数据 
}	

/*******************************************************************************
* 函 数 名         : IIC_Stop
* 函数功能		   : 产生IIC停止信号   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL(0);
	IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
 	IIC_SCL(1); 
	delay_us(2); 
	IIC_SDA(1);//发送I2C总线结束信号
	delay_us(2);							   	
}

/*******************************************************************************
* 函 数 名         : IIC_Wait_Ack
* 函数功能		   : 等待应答信号到来   
* 输    入         : 无
* 输    出         : 1，接收应答失败
        			 0，接收应答成功
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 tempTime=0;
	SDA_IN();      //SDA设置为输入  
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
	IIC_SCL(0);//时钟输出0 	   
	return 0;  
} 

/*******************************************************************************
* 函 数 名         : IIC_Ack
* 函数功能		   : 产生ACK应答  
* 输    入         : 无
* 输    出         : 无
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
* 函 数 名         : IIC_NAck
* 函数功能		   : 产生NACK非应答  
* 输    入         : 无
* 输    出         : 无
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
* 函 数 名         : IIC_Send_Byte
* 函数功能		   : IIC发送一个字节 
* 输    入         : txd：发送一个字节
* 输    出         : 无
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL(0);//拉低时钟开始数据传输
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
* 函 数 名         : IIC_Read_Byte
* 函数功能		   : IIC读一个字节 
* 输    入         : ack=1时，发送ACK，ack=0，发送nACK 
* 输    出         : 应答或非应答
*******************************************************************************/  
u8 IIC_Read_Byte(u8 ack)
{
	u8 i,receive=0;
	SDA_IN();//SDA设置为输入
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
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




