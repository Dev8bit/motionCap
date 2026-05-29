/*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* bmm050_support.c
* Date: 2016/03/17
* Revision: 1.0.6 $
*
* Usage: Sensor Driver support file for  BMM050 and BMM150 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/

/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/
#include "boards.h"
#include "app_util_platform.h"
#include "bmx055_support.h"
#include "bmm050.h"
#include "bma2x2.h"
#include "bmg160.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_timer.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_gpio.h"
#include "Adafruit_AHRS_Madgwick.h"
#include "Adafruit_AHRS_NXPFusion.h"
#include "arm_math.h"

#define LOG_SENCOR_ENABLE			0
	
#define OUTPUT_AXIS_RAWDATA			0
#define OUTPUT_QUAT_DATA			0

#define CALIBRATE_MAG_FLAG			0

#define DYNIMAIC_FUSION_ENABLE		1
#define DYNIMAIC_IMU_GYRO_FILTER	0.2f

//define quaternion data packet
#define PACKET_HEAD					'$'
#define PACKET_LENGTH   			(23)		//packet length of eMPL-pythonclient
#define PACKET_DEBUG    			(1)
#define PACKET_QUAT     			(2)
#define PACKET_DATA     			(3)


//BMX055 left-handed
#define BMM050_API
#define BMA2x2_API
#define BMG160_API

#define BMX055_SDO1_PIN				12
#define BMX055_SDO2_PIN				12
#define BMX055_CSB3_PIN				31

//Address
//#define BMM050_I2C_ADDRESS          (0x10)
//#define BMA2x2_I2C_ADDRESS          (0x18)
//#define BMG160_I2C_ADDRESS          (0x68)
//Config
#define BMA2x2_RANGE_RESOLUTION 	(2.0f)
/************** I2C/SPI buffer length ******/
#define	I2C_BUFFER_LEN 				8
#define SPI_BUFFER_LEN 				5

/* TWI instance ID. */
#define TWI_INSTANCE_ID     		0
/* TWI instance. */
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
/* TIMER2 instance ID. */
#define TIMER2_INSTANCE_ID     		2
/* TIMER2 instance ID. */
#define TIMER2_TICK_MS     			10

#define REPORT_TICK_MS     			10
/* TIMER2 instance. */
const nrf_drv_timer_t m_timer2 = NRF_DRV_TIMER_INSTANCE(TIMER2_INSTANCE_ID);

/* Indicates  notify main function starting sample sensor*/
static volatile bool m_sampleSensorEvent = false;
/* Indicates  notify main function starting sample sensor*/
static volatile bool m_sendDataEvent = false;

//X Y axis scale and bias offset
float m_xMagB = 0.0f, m_xMagS = 1.0f, m_yMagB = 0.0f, m_yMagS = 1.0f, 
	m_zMagB = 0.0f, m_zMagS = 1.0f;
//calibrated mag value	
float m_xMagC = 0.0f, m_yMagC = 0.0f, m_zMagC = 0.0f;

float m_accX, m_accY, m_accZ, m_gyroX, m_gyroY, m_gyroZ,
	m_magX, m_magY, m_magZ;
float magMaxX = 0.0, magMinX = 0.0, magMaxY = 0.0, magMinY = 0.0;
	
float quat[4] = {0.0f};


struct bma2x2_t bma2x2;
struct bmm050_t bmm050;
struct bmg160_t bmg160;

/**
 * @brief Macro for converting radian to angle degree.
 */
#define RAD2DEG 			57.2957795f
#define DEG2RAD 			0.017453f
#define PI_DEG				180.0f
#define PI_2_DEG			90.0f

#define ACC_RANGE			2.0f  //2g sensitivity
#define ACC_12BITS			2048.0f
#define ACC_MAXMEASURE		(ACC_12BITS / ACC_RANGE)
#define ACC_RESOLUTION		(ACC_RANGE / ACC_12BITS * 9.8f)

#define GYRO_RESOLUTION		(1.0f / 131.2f)
#define MAG_RESOLUTION		0.3f

void nrf_twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_BMX055_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = true
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_BMX055_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

#define UP_LMT	1
#define LOW_LMT	0

static void getlimit(bool isUp, const float* src, float* lmt)
{
	if(isUp){
		if(*src > *lmt) 
			*lmt = *src;}
	else{
		if(*src < *lmt) 
			*lmt = *src;}
}


#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
static void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,
	s16 g_x,s16 g_y,s16 g_z,
	s16 m_x,s16 m_y,s16 m_z)
{
	char data_to_send[50];
	u8 _cnt=0;
	s16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	for(uint8_t uCount = 0; uCount < _cnt; ++uCount)
	{
		NRF_LOG_RAW_INFO("%c", data_to_send[uCount]);
	}
	
	NRF_LOG_FLUSH();
}

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw)
{
	char data_to_send[50];
	u8 _cnt=0;
	s16 _temp;
	s32 _temp2 = 0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = 0;
	
	data_to_send[_cnt++] = 0;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	for(uint8_t uCount = 0; uCount < _cnt; ++uCount)
	{
		NRF_LOG_RAW_INFO("%c", data_to_send[uCount]);
	}
	
	NRF_LOG_FLUSH();
}

void nrf_send_eMPLPack(void)
{	
	ANO_DT_Send_Status(getRoll(quat[0], quat[1], quat[2], quat[3]),
	getPitch(quat[0], quat[1], quat[2], quat[3]),
	getYaw(quat[0], -quat[1], quat[2], -quat[3]));
	
//	ANO_DT_Send_Status(Adafruit_NXPSensorFusion_getRoll(),
//	Adafruit_NXPSensorFusion_getPitch(),
//	Adafruit_NXPSensorFusion_getYaw());	
		
	ANO_DT_Send_Senser(m_accX, m_accY, m_accZ, 
	m_gyroX, m_gyroY, m_gyroZ, 
	m_yMagC, m_xMagC, m_zMagC);
	
#if OUTPUT_AXIS_RAWDATA
	NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER" ", NRF_LOG_FLOAT(m_magX));
	NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER" ", NRF_LOG_FLOAT(m_magY));	
	NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(m_magZ));
	NRF_LOG_RAW_INFO("\r\n"); 

//	NRF_LOG_INFO("accX:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_accX));
//	NRF_LOG_INFO("accY:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_accY));
//	NRF_LOG_INFO("accZ:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_accZ));
//	NRF_LOG_INFO("m_gyroX:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_gyroX));
//	NRF_LOG_INFO("m_gyroY:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_gyroY));
//	NRF_LOG_INFO("m_gyroZ:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_gyroZ));
//	NRF_LOG_INFO("magX:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_magX));
//	NRF_LOG_INFO("magY:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_magY));
//	NRF_LOG_INFO("magZ:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_magZ));

	//calibrate magnetmeter
//	{
//		NRF_LOG_INFO("magMaxX:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(magMaxX));
//		NRF_LOG_INFO("magMinX:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(magMinX));
//		NRF_LOG_INFO("magMaxY:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(magMaxY));
//		NRF_LOG_INFO("magMinY:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(magMinY));
//		
//		if(fabs(magMaxX-magMinX) > fabs(magMaxY-magMinY))
//		{
//			m_xMagS = 1.0f;
//			m_yMagS = (float)(magMaxX - magMinX) / (magMaxY - magMinY);		
//		}
//		else
//		{
//			m_xMagS = (float)(magMaxY - magMinY) / (magMaxX - magMinX);
//			m_yMagS = 1.0f;				
//		}

//		m_xMagB = (0.5f * (magMaxX - magMinX) - magMaxX) * m_xMagS;
//		m_yMagB = (0.5f * (magMaxY - magMinY) - magMaxY) * m_yMagS;
//		
//		NRF_LOG_INFO("m_xMagS:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_xMagS));
//		NRF_LOG_INFO("m_xMagB:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_xMagB));
//		NRF_LOG_INFO("m_yMagS:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_yMagS));
//		NRF_LOG_INFO("m_yMagB:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_yMagB));
//	}
#endif

#if	OUTPUT_QUAT_DATA

    char out[PACKET_LENGTH];
	int32_t qData[4] = {0};
	float32_t fData[4] = {q0, q1, q2, q3};
	
	arm_float_to_q31(fData, qData, 4);
	
	//q31 to q30
	qData[0] = qData[0] >> 1;
	qData[1] = -(qData[1] >> 1);
	qData[2] = qData[2] >> 1;
	qData[3] = -(qData[3] >> 1);
	
	//convert long to char
    out[0] = PACKET_HEAD;
    out[1] = PACKET_QUAT;
	
    out[3] = (char)(qData[0] >> 24);
    out[4] = (char)(qData[0] >> 16);
    out[5] = (char)(qData[0] >> 8);
    out[6] = (char)qData[0];
	
    out[7] = (char)(qData[1] >> 24);
    out[8] = (char)(qData[1] >> 16);
    out[9] = (char)(qData[1] >> 8);
    out[10] = (char)qData[1];
	
    out[11] = (char)(qData[2] >> 24);
    out[12] = (char)(qData[2] >> 16);
    out[13] = (char)(qData[2] >> 8);
    out[14] = (char)qData[2];
	
    out[15] = (char)(qData[3] >> 24);
    out[16] = (char)(qData[3] >> 16);
    out[17] = (char)(qData[3] >> 8);
    out[18] = (char)qData[3];

    out[21] = 0x0D;
    out[22] = 0x0A;

	for(uint8_t uCount = 0; uCount < PACKET_LENGTH; ++uCount)
	{
		NRF_LOG_RAW_INFO("%c", out[uCount]);
	}
	
#endif
	NRF_LOG_FLUSH();
}



/**
 * @brief Handler for timer events.
 */
static void timer_sample_event_handler(nrf_timer_event_t event_type, void* p_context)
{
	static uint16_t usartRstCount = 0;
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
			m_sampleSensorEvent = true;
		
			if(usartRstCount++ > (REPORT_TICK_MS / TIMER2_TICK_MS))
			{
				m_sendDataEvent = true;
				usartRstCount = 0;
			}
            break;

        default:
            //Do nothing.
            break;
    }
}

void nrf_SampleBMX055_loopRoutine(void)
{
	if(m_sampleSensorEvent)
	{
		BMX055_updateFusion();		//exec rate 100Hz
		Adafruit_NXPSensorFusion_getQuat(quat);
		m_sampleSensorEvent = false;			
	}

#if LOG_SENCOR_ENABLE
	if(m_sendDataEvent)
	{
		nrf_send_eMPLPack();	
		m_sendDataEvent = false;
	}	
#endif
}

void nrf_timer_SampleBMX055_init(void)
{
    uint32_t time_ms = TIMER2_TICK_MS; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;

    //Configure m_timer2 for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&m_timer2, &timer_cfg, timer_sample_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&m_timer2, time_ms);

    nrf_drv_timer_extended_compare(
         &m_timer2, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&m_timer2);	
}

/*----------------------------------------------------------------------------*
*	The following functions are used for reading and writing of
*	sensor data using I2C or SPI communication
*----------------------------------------------------------------------------*/
#ifdef BMA2x2_API
 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *               will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *               which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BMA2x2_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *              will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMA2x2_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/* \Brief: The function is used as SPI bus write
 * \Return : Status of the SPI write
 * \param dev_addr : The device address of the sensor
 * \param reg_addr : Address of the first register,
 *      will data is going to be written
 * \param reg_data : It is a value hold in the array,
 *	will be used for write the value into the register
 * \param cnt : The no of byte of data to be write
 */
s8 BMA2x2_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/* \Brief: The function is used as SPI bus read
 * \Return : Status of the SPI read
 * \param dev_addr : The device address of the sensor
 * \param reg_addr : Address of the first register,
 *   will data is going to be read
 * \param reg_data : This data read from the sensor, which is hold in an array
 * \param cnt : The no of byte of data to be read */
s8 BMA2x2_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
 * \Brief: SPI/I2C init routine
*/
s8 BMA2x2_I2C_routine(void);
s8 BMA2x2_SPI_routine(void);
#endif
/********************End of I2C/SPI function declarations*******************/
/*	Brief : The delay routine
 *	\param : delay in ms
 */
void BMA2x2_delay_msek(u32 msek);
/*!
 *	@brief This function is an example for delay
 *	@param : None
 *	@return : communication result
 */

#ifdef BMA2x2_API
/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bma2x2_t
*-------------------------------------------------------------------------*/
s8 BMA2x2_I2C_routine(void)
{
/*--------------------------------------------------------------------------*
 *  By using bma2x2 the following structure parameter can be accessed
 *	Bus write function pointer: BMA2x2_WR_FUNC_PTR
 *	Bus read function pointer: BMA2x2_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	bma2x2.bus_write = BMA2x2_I2C_bus_write;
	bma2x2.bus_read = BMA2x2_I2C_bus_read;
	bma2x2.delay_msec = BMA2x2_delay_msek;
	bma2x2.dev_addr = BMA2x2_I2C_ADDR1;

	return BMA2x2_INIT_VALUE;
}

/*---------------------------------------------------------------------------*
 * The following function is used to map the SPI bus read, write and delay
 * with global structure bma2x2_t
 *--------------------------------------------------------------------------*/
//s8 BMA2x2_SPI_routine(void)
//{
///*--------------------------------------------------------------------------*
// *  By using bma2x2 the following structure parameter can be accessed
// *	Bus write function pointer: BMA2x2_WR_FUNC_PTR
// *	Bus read function pointer: BMA2x2_RD_FUNC_PTR
// *	Delay function pointer: delay_msec
// *--------------------------------------------------------------------------*/

//	bma2x2.bus_write = BMA2x2_SPI_bus_write;
//	bma2x2.bus_read = BMA2x2_SPI_bus_read;
//	bma2x2.delay_msec = BMA2x2_delay_msek;

//	return BMA2x2_INIT_VALUE;
//}

#define BMA2x2_BUS_READ_WRITE_ARRAY_INDEX	1
#define BMA2x2_SPI_BUS_WRITE_CONTROL_BYTE	0x7F
#define BMA2x2_SPI_BUS_READ_CONTROL_BYTE	0x80
/*-------------------------------------------------------------------*
*	This is a sample code for read and write the data by using I2C/SPI
*	Use either I2C or SPI based on your need
*
*-----------------------------------------------------------------------*/
/*	For configuring the I2C it is required to switch ON
 *	SDI, SDO and CLk and also select the device address
 * The following definition of I2C address is used for the following sensors
 * BMA255
 * BMA253
 * BMA355
 * BMA280
 * BMA282
 * BMA223
 * BMA254
 * BMA284
 * BMA250E
 * BMA222E

 #define BMA2x2_I2C_ADDR1         0x18
 #define BMA2x2_I2C_ADDR2         0x19

 * The following definition of I2C address is used for the following sensors
 * BMC150
 * BMC056
 * BMC156

 #define BMA2x2_I2C_ADDR3        0x10
 #define BMA2x2_I2C_ADDR4        0x11
 *************************************************************************/
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *              will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMA2x2_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMA2x2_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = BMA2x2_INIT_VALUE;

	array[BMA2x2_INIT_VALUE] = reg_addr;
	for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++) {
		array[stringpos + BMA2x2_BUS_READ_WRITE_ARRAY_INDEX] =
		*(reg_data + stringpos);
	}
	/*
	* Please take the below function as your reference for
	* write the data using I2C communication
	* "IERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write function here
	* iError is an return value of I2C read function
	* Please select your valid return value
	* In the driver SUCCESS defined as 0
	* and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation
	* done in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
	iError = nrf_drv_twi_tx(&m_twi, dev_addr, array, cnt + 1, false);

	return (s8)iError;
}

 /*   \Brief: The function is used as I2C bus read
 *    \Return : Status of the I2C read
 *    \param dev_addr : The device address of the sensor
 *    \param reg_addr : Address of the first register,
 *            will data is going to be read
 *    \param reg_data : This data read from the sensor,
 *            which is hold in an array
 *    \param cnt : The no of byte of data to be read
 */
s8 BMA2x2_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMA2x2_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN] = {BMA2x2_INIT_VALUE};
	u8 stringpos = BMA2x2_INIT_VALUE;

	array[BMA2x2_INIT_VALUE] = reg_addr;
	/* Please take the below function as your reference
	 * for read the data using I2C communication
	 * add your I2C rad function here.
	 * "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
	 * iError is an return value of SPI write function
	 * Please select your valid return value
     * In the driver SUCCESS defined as 0
     * and FAILURE defined as -1
	 */
	iError = nrf_drv_twi_tx(&m_twi, dev_addr, array, 1, true);
	iError = nrf_drv_twi_rx(&m_twi, dev_addr, array, cnt);
	
	for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++)
		*(reg_data + stringpos) = array[stringpos];
	return (s8)iError;
}

/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *          will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *          which is hold in an array
 *	\param cnt : The no of byte of data to be read */
//s8 BMA2x2_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
//{
//	s32 iError = BMA2x2_INIT_VALUE;
//	u8 array[SPI_BUFFER_LEN] = {0xFF};
//	u8 stringpos;
//	/*	For the SPI mode only 7 bits of register addresses are used.
//	The MSB of register address is declared the bit what functionality it is
//	read/write (read as 1/write as 0)*/
//	array[BMA2x2_INIT_VALUE] = reg_addr|BMA2x2_SPI_BUS_READ_CONTROL_BYTE;
//	/*read routine is initiated register address is mask with 0x80*/
//	/*
//	* Please take the below function as your reference for
//	* read the data using SPI communication
//	* " IERROR = SPI_READ_WRITE_STRING(ARRAY, ARRAY, CNT+1)"
//	* add your SPI read function here
//	* iError is an return value of SPI read function
//	* Please select your valid return value
//	* In the driver SUCCESS defined as 0
//	* and FAILURE defined as -1
//	* Note :
//	* This is a full duplex operation,
//	* The first read data is discarded, for that extra write operation
//	* have to be initiated. For that cnt+1 operation done in the SPI read
//	* and write string function
//	* For more information please refer data sheet SPI communication:
//	*/
//	for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++) {
//		*(reg_data + stringpos) = array[stringpos +
//		BMA2x2_BUS_READ_WRITE_ARRAY_INDEX];
//	}
//	return (s8)iError;
//}

/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
*               will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
//s8 BMA2x2_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
//{
//	s32 iError = BMA2x2_INIT_VALUE;
//	u8 array[SPI_BUFFER_LEN * 2];
//	u8 stringpos = BMA2x2_INIT_VALUE;

//	for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++) {
//		/* the operation of (reg_addr++)&0x7F done:
//		because it ensure the
//		0 and 1 of the given value
//		It is done only for 8bit operation*/
//		array[stringpos * 2] = (reg_addr++) &
//		BMA2x2_SPI_BUS_WRITE_CONTROL_BYTE;
//		array[stringpos * 2 + BMA2x2_BUS_READ_WRITE_ARRAY_INDEX] =
//		*(reg_data + stringpos);
//	}
//	/* Please take the below function as your reference
//	 * for write the data using SPI communication
//	 * add your SPI write function here.
//	 * "IERROR = SPI_WRITE_STRING(ARRAY, CNT*2)"
//	 * iError is an return value of SPI write function
//	 * Please select your valid return value
//	 * In the driver SUCCESS defined as 0
//     * and FAILURE defined as -1
//	 */
//	return (s8)iError;
//}

/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMA2x2_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
	nrf_delay_ms(msek);
}
#endif

#ifdef BMM050_API
/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *        will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *        which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BMM050_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *              will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMM050_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *              will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
//s8 BMM050_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *             will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *             which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
//s8 BMM050_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
 * \Brief: SPI/I2C init routine
*/
s8 BMM050_I2C_routine(void);
//s8 BMM050_SPI_routine(void);
#endif
/********************End of I2C/SPI function declarations*********************/
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMM050_delay_msek(u32 msek);
#ifdef BMM050_API
/*--------------------------------------------------------------------------*/
/*	The following function is used to map the I2C bus read, write, delay and
 *	device address with global structure bmm050
 *-------------------------------------------------------------------------*/
s8 BMM050_I2C_routine(void)
{
/*--------------------------------------------------------------------------*/
/*	By using bmm050 the following structure parameter can be accessed
 *	Bus write function pointer: BMM050_WR_FUNC_PTR
 *	Bus read function pointer: BMM050_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	bmm050.bus_write = BMM050_I2C_bus_write;
	bmm050.bus_read = BMM050_I2C_bus_read;
	bmm050.delay_msec = BMM050_delay_msek;
	bmm050.dev_addr = BMM050_I2C_ADDRESS;

	return BMM050_INIT_VALUE;
}

/*---------------------------------------------------------------------------*/
/*	The following function is used to map the SPI bus read, write and delay
 *	with global structure bmm050
 *--------------------------------------------------------------------------*/
//s8 BMM050_SPI_routine(void)
//{
///*--------------------------------------------------------------------------*
// *  By using bmm050 the following structure parameter can be accessed
// *	Bus write function pointer: BMM050_WR_FUNC_PTR
// *	Bus read function pointer: BMM050_RD_FUNC_PTR
// *	Delay function pointer: delay_msec
// *--------------------------------------------------------------------------*/

//	bmm050.bus_write = BMM050_SPI_bus_write;
//	bmm050.bus_read = BMM050_SPI_bus_read;
//	bmm050.delay_msec = BMM050_delay_msek;

//	return BMM050_INIT_VALUE;
//}

#define MASK_DATA1	0xFF
#define MASK_DATA2	0x80
#define MASK_DATA3	0x7F
#define	C_BMM050_ONE_U8X	(1)
#define	C_BMM050_TWO_U8X	(2)
/*-------------------------------------------------------------------*
*
*	This is a sample code for read and write the data by using I2C/SPI
*	Use either I2C or SPI based on your need
*	The device address defined in the bmm050.h file
*
*-----------------------------------------------------------------------*/
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *              will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMM050_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMM050_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = BMM050_INIT_VALUE;

	array[BMM050_INIT_VALUE] = reg_addr;
	for (stringpos = BMM050_INIT_VALUE; stringpos < cnt; stringpos++)
		array[stringpos + C_BMM050_ONE_U8X] = *(reg_data + stringpos);
	/*
	* Please take the below function as your reference for
	* write the data using I2C communication
	* "IERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+C_BMM050_ONE_U8X)"
	* add your I2C write function here
	* iError is an return value of I2C read function
	* Please select your valid return value
	* In the driver SUCCESS defined as BMM050_INIT_VALUE
    * and FAILURE defined as -C_BMM050_ONE_U8X
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+C_BMM050_ONE_U8X operation done
	* in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
	iError = nrf_drv_twi_tx(&m_twi, dev_addr, array, cnt + 1, false);
	return (s8)iError;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *            will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *            which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BMM050_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMM050_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN] = {BMM050_INIT_VALUE};
	u8 stringpos = BMM050_INIT_VALUE;

	array[BMM050_INIT_VALUE] = reg_addr;
	/* Please take the below function as your reference
	 * for read the data using I2C communication
	 * add your I2C rad function here.
	 * "IERROR = I2C_WRITE_READ_STRING(
	 *  DEV_ADDR, ARRAY, ARRAY, C_BMM050_ONE_U8X, CNT)"
	 * iError is an return value of SPI write function
	 * Please select your valid return value
	 * In the driver SUCCESS defined as BMM050_INIT_VALUE
     * and FAILURE defined as -C_BMM050_ONE_U8X
	 */
	iError = nrf_drv_twi_tx(&m_twi, dev_addr, array, 1, true);
	iError = nrf_drv_twi_rx(&m_twi, dev_addr, array, cnt);
	
	for (stringpos = BMM050_INIT_VALUE; stringpos < cnt; stringpos++)
		*(reg_data + stringpos) = array[stringpos];

	return (s8)iError;
}

/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *		will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *              which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
//s8 BMM050_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
//{
//	s32 iError = BMM050_INIT_VALUE;
//	u8 array[SPI_BUFFER_LEN] = {MASK_DATA1};
//	u8 stringpos;
//	/*	For the SPI mode only 7 bits of register addresses are used.
//	The MSB of register address is declared the bit what functionality it is
//	read/write (read as C_BMM050_ONE_U8X/write as BMM050_INIT_VALUE)*/
//	/*read routine is initiated register address is mask with 0x80*/
//	array[BMM050_INIT_VALUE] = reg_addr|MASK_DATA2;
//	/*
//	* Please take the below function as your reference for
//	* read the data using SPI communication
//	* " IERROR = SPI_READ_WRITE_STRING(ARRAY, ARRAY, CNT+C_BMM050_ONE_U8X)"
//	* add your SPI read function here
//	* iError is an return value of SPI read function
//	* Please select your valid return value
//	* In the driver SUCCESS defined as BMM050_INIT_VALUE
//	* and FAILURE defined as -1
//	* Note :
//	* This is a full duplex operation,
//	* The first read data is discarded, for that extra write operation
//	* have to be initiated. For that cnt+C_BMM050_ONE_U8X operation done
//	* in the SPI read
//	* and write string function
//	* For more information please refer data sheet SPI communication:
//	*/
//	for (stringpos = BMM050_INIT_VALUE; stringpos < cnt; stringpos++)
//		*(reg_data + stringpos) = array[stringpos+C_BMM050_ONE_U8X];

//	return (s8)iError;
//}

/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *		will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
//s8 BMM050_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
//{
//	s32 iError = BMM050_INIT_VALUE;
//	u8 array[SPI_BUFFER_LEN * C_BMM050_TWO_U8X];
//	u8 stringpos = BMM050_INIT_VALUE;

//	for (stringpos = BMM050_INIT_VALUE; stringpos < cnt; stringpos++) {
//		/* the operation of (reg_addr++)&0x7F done: because it ensure
//	the BMM050_INIT_VALUE and C_BMM050_ONE_U8X of the given value
//	It is done only for 8bit operation*/
//		array[stringpos * C_BMM050_TWO_U8X] = (reg_addr++) & MASK_DATA3;
//		array[stringpos * C_BMM050_TWO_U8X + C_BMM050_ONE_U8X] =
//		*(reg_data + stringpos);
//	}
//	/* Please take the below function as your reference
//	 * for write the data using SPI communication
//	 * add your SPI write function here.
//	 * "IERROR = SPI_WRITE_STRING(ARRAY, CNT*C_BMM050_TWO_U8X)"
//	 * iError is an return value of SPI write function
//	 * Please select your valid return value
//     * In the driver SUCCESS defined as BMM050_INIT_VALUE
//     * and FAILURE defined as -1
//	 */
//	return (s8)iError;
//}
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMM050_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
	nrf_delay_ms(msek);
}
#endif

#ifdef BMG160_API
/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BMG160_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMG160_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
//s8 BMG160_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read */
//s8 BMG160_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
 * \Brief: SPI/I2C init routine
*/
s8 BMG160_I2C_routine(void);
//s8 BMG160_SPI_routine(void);
#endif
/********************End of I2C/SPI function declarations***********************/
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMG160_delay_msek(u32 msek);
#ifdef BMG160_API
/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bmg160_t
*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*
 *  By using bmg160 the following structure parameter can be accessed
 *	Bus write function pointer: BMG160_WR_FUNC_PTR
 *	Bus read function pointer: BMG160_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
 s8 BMG160_I2C_routine(void) {

	bmg160.bus_write = BMG160_I2C_bus_write;
	bmg160.bus_read = BMG160_I2C_bus_read;
	bmg160.delay_msec = BMG160_delay_msek;
	bmg160.dev_addr = BMG160_I2C_ADDR1;

	return BMG160_INIT_VALUE;
}

/*---------------------------------------------------------------------------*
 *	The following function is used to map the SPI bus read, write and delay
 *	with global structure bmg160_t
 *--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*
 *  By using bmg160 the following structure parameter can be accessed
 *	Bus write function pointer: BMG160_WR_FUNC_PTR
 *	Bus read function pointer: BMG160_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *-------------------------------------------------------------------------*/
//s8 BMG160_SPI_routine(void) {

//	bmg160.bus_write = BMG160_SPI_bus_write;
//	bmg160.bus_read = BMG160_SPI_bus_read;
//	bmg160.delay_msec = BMG160_delay_msek;

//	return BMG160_INIT_VALUE;
//}

#define MASK_DATA1	0xFF
#define MASK_DATA2	0x80
#define MASK_DATA3	0x7F
#define	C_BMG160_ONE_U8X	(1)
#define	C_BMG160_TWO_U8X	(2)
/*-------------------------------------------------------------------*
*
*	This is a sample code for read and write the data by using I2C/SPI
*	Use either I2C or SPI based on your need
*	The device address defined in the bmg160.h file
*
*-----------------------------------------------------------------------*/
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMG160_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMG160_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = BMG160_INIT_VALUE;
	array[BMG160_INIT_VALUE] = reg_addr;
	for (stringpos = BMG160_INIT_VALUE; stringpos < cnt; stringpos++) {
		array[stringpos + BMG160_GEN_READ_WRITE_DATA_LENGTH] = *(reg_data + stringpos);
	}
	/*
	* Please take the below function as your reference for
	* write the data using I2C communication
	* "IERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write function here
	* iError is an return value of I2C read function
	* Please select your valid return value
	* In the driver SUCCESS defined as BMG160_INIT_VALUE
    * and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
	iError = nrf_drv_twi_tx(&m_twi, dev_addr, array, cnt + 1, false);
	return (s8)iError;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of data to be read
 */
s8 BMG160_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMG160_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN] = {BMG160_INIT_VALUE};
	u8 stringpos = BMG160_INIT_VALUE;
	array[BMG160_INIT_VALUE] = reg_addr;
	/* Please take the below function as your reference
	 * for read the data using I2C communication
	 * add your I2C rad function here.
	 * "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
	 * iError is an return value of SPI write function
	 * Please select your valid return value
	 * In the driver SUCCESS defined as BMG160_INIT_VALUE
     * and FAILURE defined as -1
	 */
	iError = nrf_drv_twi_tx(&m_twi, dev_addr, array, 1, true);
	iError = nrf_drv_twi_rx(&m_twi, dev_addr, array, cnt);
	
	for (stringpos = BMG160_INIT_VALUE; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = array[stringpos];
	}
	return (s8)iError;
}

/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of data to be read
 */
//s8 BMG160_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
//{
//	s32 iError=BMG160_INIT_VALUE;
//	u8 array[SPI_BUFFER_LEN]={MASK_DATA1};
//	u8 stringpos;
//	/*	For the SPI mode only 7 bits of register addresses are used.
//	The MSB of register address is declared the bit what functionality it is
//	read/write (read as 1/write as BMG160_INIT_VALUE)*/
//	array[BMG160_INIT_VALUE] = reg_addr|MASK_DATA2;/*read routine is initiated register address is mask with 0x80*/
//	/*
//	* Please take the below function as your reference for
//	* read the data using SPI communication
//	* " IERROR = SPI_READ_WRITE_STRING(ARRAY, ARRAY, CNT+1)"
//	* add your SPI read function here
//	* iError is an return value of SPI read function
//	* Please select your valid return value
//	* In the driver SUCCESS defined as BMG160_INIT_VALUE
//    * and FAILURE defined as -1
//	* Note :
//	* This is a full duplex operation,
//	* The first read data is discarded, for that extra write operation
//	* have to be initiated. For that cnt+1 operation done in the SPI read
//	* and write string function
//	* For more information please refer data sheet SPI communication:
//	*/
//	for (stringpos = BMG160_INIT_VALUE; stringpos < cnt; stringpos++) {
//		*(reg_data + stringpos) = array[stringpos+BMG160_GEN_READ_WRITE_DATA_LENGTH];
//	}
//	return (s8)iError;
//}

/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
//s8 BMG160_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
//{
//	s32 iError = BMG160_INIT_VALUE;
//	u8 array[SPI_BUFFER_LEN * C_BMG160_TWO_U8X];
//	u8 stringpos = BMG160_INIT_VALUE;
//	for (stringpos = BMG160_INIT_VALUE; stringpos < cnt; stringpos++) {
//		/* the operation of (reg_addr++)&0x7F done: because it ensure the
//		   BMG160_INIT_VALUE and 1 of the given value
//		   It is done only for 8bit operation*/
//		array[stringpos * C_BMG160_TWO_U8X] = (reg_addr++) & MASK_DATA3;
//		array[stringpos * C_BMG160_TWO_U8X + BMG160_GEN_READ_WRITE_DATA_LENGTH] = *(reg_data + stringpos);
//	}
//	/* Please take the below function as your reference
//	 * for write the data using SPI communication
//	 * add your SPI write function here.
//	 * "IERROR = SPI_WRITE_STRING(ARRAY, CNT*2)"
//	 * iError is an return value of SPI write function
//	 * Please select your valid return value
//	 * In the driver SUCCESS defined as BMG160_INIT_VALUE
//     * and FAILURE defined as -1
//	 */
//	return (s8)iError;
//}

/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMG160_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
	nrf_delay_ms(msek);
}
#endif


char BMX055_init(void)
{
	/* status of communication*/
	s8 com_rslt = SUCCESS;
	
	//set BMX055 IIC address
	nrf_gpio_cfg_input(BMX055_SDO1_PIN, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(BMX055_SDO2_PIN, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(BMX055_CSB3_PIN, NRF_GPIO_PIN_PULLDOWN);
	
	//assignment of BMX055 function pointer with IIC communication function
	BMA2x2_I2C_routine();
	BMM050_I2C_routine();
	BMG160_I2C_routine();
	
	//initialize modules of BMX055 
	com_rslt += bmg160_init(&bmg160);	
	com_rslt += bma2x2_init(&bma2x2);
	com_rslt += bmm050_init(&bmm050);
	
	APP_ERROR_CHECK(com_rslt);
	
	com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
	com_rslt += bmg160_set_power_mode(BMG160_MODE_NORMAL);
	com_rslt += bmm050_set_functional_state(BMM050_NORMAL_MODE);
	
	/* set bandwidth of 250Hz*/
	com_rslt += bma2x2_set_bw(BMA2x2_BW_250HZ);	
	/* set accelerometer range 2G*/
	com_rslt += bma2x2_set_range(BMA2x2_RANGE_2G);	
	
	/* set gyro bandwidth of 230Hz*/
	com_rslt += bmg160_set_bw(BMG160_BW_230_HZ);
	/* set gyro range 250 degree per second*/
	com_rslt += bmg160_set_range_reg(BMG160_RANGE_250);

	/* set magnetomenter data rate of 30Hz*/
	com_rslt += bmm050_set_data_rate(BMM050_DATA_RATE_25HZ);

	return com_rslt;
}

char BMX055_axisCompensation(void)
{
	s8 com_rslt = SUCCESS;
	
	//set accelerometer slow compensation
	com_rslt +=	bma2x2_set_slow_comp(0, 1);
	com_rslt +=	bma2x2_set_slow_comp(1, 1);
	com_rslt +=	bma2x2_set_slow_comp(2, 1);
	
	com_rslt +=	bmg160_set_offset_unfilt(BMG160_SLOW_OFFSET, BMG160_ENABLE);
	com_rslt +=	bmg160_set_slow_offset_enable_axis(0, 1);
	com_rslt +=	bmg160_set_slow_offset_enable_axis(1, 1);
	com_rslt +=	bmg160_set_slow_offset_enable_axis(2, 1);
	
	return com_rslt;
}

char BMX055_calibrateMag(char bConfigMag)
{
	/* Structure used for read the mag xyz data*/
	struct bmm050_mag_data_s16_t tMagData;
	s16 xMagMax = 0x8000, xMagMin = 0x7fff, yMagMax= 0x8000, yMagMin = 0x7fff;
	u16 NumConfig = 0;
	s8 com_rslt = SUCCESS;
	
	if(!bConfigMag)
		return com_rslt;
	
	NRF_LOG_INFO("start calibrating magnetometer...\r\n");
	NRF_LOG_FLUSH();
	nrf_delay_ms(500);
	
	while(bConfigMag)
	{
		com_rslt += bmm050_read_mag_data_XYZ(&tMagData);
		if(tMagData.datax > xMagMax) xMagMax = tMagData.datax;
		if(tMagData.datax < xMagMin) xMagMin = tMagData.datax;
		if(tMagData.datay > yMagMax) yMagMax = tMagData.datay;
		if(tMagData.datay < yMagMin) yMagMin = tMagData.datay;
		
		if(++NumConfig < 400)	//sample 6 sec
		{
			nrf_delay_ms(15); 	//sample rate >= 60Hz(double magnetometer data sample rate)
		}
		else
		{
			bConfigMag = false;
			break;
		}
	}
	
	if(abs(xMagMax-xMagMin) > abs(yMagMax-yMagMin))
	{
		m_xMagS = 1.0f;
		m_yMagS = (float)(xMagMax - xMagMin) / (yMagMax - yMagMin);		
	}
	else
	{
		m_xMagS = (float)(yMagMax - yMagMin) / (xMagMax - xMagMin);
		m_yMagS = 1.0f;				
	}

	m_xMagB = (0.5f * (xMagMax - xMagMin) - xMagMax) * m_xMagS;
	m_yMagB = (0.5f * (yMagMax - yMagMin) - yMagMax) * m_yMagS;
	
	NRF_LOG_INFO("m_xMagS:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_xMagS));
	NRF_LOG_INFO("m_xMagB:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_xMagB));
	NRF_LOG_INFO("m_yMagS:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_yMagS));
	NRF_LOG_INFO("m_yMagB:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_yMagB));
	NRF_LOG_FLUSH();
	return com_rslt;
}

void BMX055_setMagCalib(float xMagS, float xMagB, float yMagS, float yMagB, float zMagS, float zMagB)
{
	m_xMagS = xMagS;
	m_yMagS = yMagS;
	m_zMagS = zMagS;
	m_xMagB = xMagB;
	m_yMagB = yMagB;
	m_zMagB = zMagB;
}

void BMX055_getMagCalib(float* xMagS, float* xMagB, float* yMagS, float* yMagB, float* zMagS, float* zMagB)
{
	*xMagS = m_xMagS;
	*yMagS = m_yMagS;
	*zMagS = m_zMagS;
	*xMagB = m_xMagB;
	*yMagB = m_yMagB;
	*zMagB = m_zMagB;
}

void BMX055_updateFusion(void)
{
	struct bmg160_data_t data_gyro;
	struct bma2x2_accel_data data_acc;
	struct bmm050_mag_data_s16_t data_mag;
	
	bma2x2_read_accel_xyz(&data_acc);
	bmg160_get_data_XYZ(&data_gyro);
	bmm050_read_mag_data_XYZ(&data_mag);

	m_accX = data_acc.x * ACC_RESOLUTION;
	m_accY = data_acc.y * ACC_RESOLUTION;
	m_accZ = data_acc.z * ACC_RESOLUTION;
	
	m_gyroX = data_gyro.datax * DEG2RAD * GYRO_RESOLUTION;
	m_gyroY = data_gyro.datay * DEG2RAD * GYRO_RESOLUTION;
	m_gyroZ = data_gyro.dataz * DEG2RAD * GYRO_RESOLUTION;
	
	m_xMagC = (data_mag.datax * MAG_RESOLUTION - m_xMagB) / m_xMagS * 100;
	m_yMagC = (data_mag.datay * MAG_RESOLUTION - m_yMagB) / m_yMagS * -100;
	m_zMagC = (data_mag.dataz * MAG_RESOLUTION - m_zMagB) / m_zMagS * 100;

	Adafruit_NXPSensorFusion_update(data_gyro.datax * GYRO_RESOLUTION, data_gyro.datay * GYRO_RESOLUTION, data_gyro.dataz * GYRO_RESOLUTION,
	data_acc.x * ACC_RESOLUTION, data_acc.y * ACC_RESOLUTION, data_acc.z * ACC_RESOLUTION,
	m_yMagC, m_xMagC, m_zMagC);
}

float getScreenAxisX(void)
{
	return getYaw(quat[0], -quat[1], quat[2], -quat[3]);
}
float getScreenAxisY(void)
{
	return getPitch(quat[0], quat[1], quat[2], quat[3]);
}

float getPitch(float q0, float q1, float q2, float q3)
{
	return asin(2*q2*q3 + 2*q0*q1) * RAD2DEG;
}

float getYaw(float q0, float q1, float q2, float q3) 
{
	return -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * RAD2DEG - 90;
}

float getRoll(float q0, float q1, float q2, float q3)
{
	return -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * RAD2DEG;
}


