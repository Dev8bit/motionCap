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
#include "bmx055_support.h"
#include "bmm050.h"
#include "bma2x2.h"
#include "bmg160.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_gpio.h"
#include "MahonyAHRS.h"
#include "arm_math.h"

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

/* TWI instance. */
extern const nrf_drv_twi_t m_twi;

struct bma2x2_t bma2x2;
struct bmm050_t bmm050;
struct bmg160_t bmg160;

//X Y axis scale and bias offset
float xMagB = -130.0f, xMagS = 1.0f, yMagB = -75.50f, yMagS = 0.902355f;

#define RAD2DEG 	57.295780f
#define DEG2RAD 	0.017453f

#define GYRO_RESOLUTION		1000.0f / 30500.0f			//dynamic
#define ACC_RESOLUTION		4.0f / 2048.0f			//dynamic
#define MAG_RESOLUTION		0.3f					//Fixed value refered Datasheet

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


char BMX055_config(void)
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
	NRF_LOG_INFO("\r\nbma2x2_ID:0x%02X\tbmg160_ID:0x%02X\tbmm050_ID:0x%02X", 
	bma2x2.chip_id, bmg160.chip_id, bmm050.company_id);
	
	com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
	com_rslt += bmg160_set_power_mode(BMG160_MODE_NORMAL);
	com_rslt += bmm050_set_functional_state(BMM050_NORMAL_MODE);
	
	/* set bandwidth of ?Hz*/
	com_rslt += bma2x2_set_bw(BMA2x2_BW_250HZ);	
	/* set accelerometer range ?G*/
	com_rslt += bma2x2_set_range(BMA2x2_RANGE_4G);	
	
	/* set gyro bandwidth of ?Hz*/
	com_rslt += bmg160_set_bw(C_BMG160_BW_230HZ_U8X);
	/* set gyro range ? degree per second*/
	com_rslt += bmg160_set_range_reg(BMG160_RANGE_1000);

	/* set magnetomenter data rate of 30Hz*/
	com_rslt += bmm050_set_data_rate(BMM050_DATA_RATE_30HZ);

	return com_rslt;
}

char BMX055_CalOrientation(void)
{
	s8 com_rslt = SUCCESS;
	
	return com_rslt;
}

char BMX055_CalibrateMag(char bConfigMag)
{
	/* Structure used for read the mag xyz data*/
	struct bmm050_mag_data_s16_t tMagData;
	s16 xMagMax = 0x8000, xMagMin = 0x7fff, yMagMax= 0x8000, yMagMin = 0x7fff;
	u16 NumConfig = 0;
	s8 com_rslt = SUCCESS;
	
	if(!bConfigMag)
		return com_rslt;
	
	while(bConfigMag)
	{
		com_rslt += bmm050_read_mag_data_XYZ(&tMagData);
		if(tMagData.datax > xMagMax) xMagMax = tMagData.datax;
		if(tMagData.datax < xMagMin) xMagMin = tMagData.datax;
		if(tMagData.datay > yMagMax) yMagMax = tMagData.datay;
		if(tMagData.datay < yMagMin) yMagMin = tMagData.datay;
		
		if(++NumConfig < 800)	//sample 12 sec
		{
			nrf_delay_ms(15); 	//sample rate >= 60Hz(double magnetometer data sample rate)
		}
		else
		{
			bConfigMag = false;
			break;
		}
	}
	
	xMagS = 1.0f;
	yMagS = (float)(xMagMax - xMagMin) / (yMagMax - yMagMin);
	xMagB = -0.5f * (xMagMax + xMagMin) * xMagS;
	yMagB = -0.5f * (yMagMax + yMagMin) * yMagS;
	
	return com_rslt;
}

void BMX055_updateAHRS(void)
{
	/* structure used for read the sensor data - xyz*/
	struct bmg160_data_t data_gyro;
	/* bma2x2acc_data structure used to read accel xyz data*/
	struct bma2x2_accel_data data_acc;
	/* Structure used for read the mag xyz data*/
	struct bmm050_mag_data_s16_t data_mag;
	float xMagC = 0.0f, yMagC = 0.0f;
	

	bmg160_get_data_XYZ(&data_gyro);
	bma2x2_read_accel_xyz(&data_acc);
	/* Intermittent reading read mag value*/
	bmm050_read_mag_data_XYZ(&data_mag);
	
	xMagC = data_mag.datax + xMagB;
	yMagC = data_mag.datay * yMagS + yMagB;
	
	MahonyAHRSupdate(data_gyro.datax * DEG2RAD * GYRO_RESOLUTION, data_gyro.datay * DEG2RAD * GYRO_RESOLUTION, data_gyro.dataz * DEG2RAD * GYRO_RESOLUTION,
	data_acc.x * ACC_RESOLUTION, data_acc.y * ACC_RESOLUTION, data_acc.z * ACC_RESOLUTION,
	xMagC * MAG_RESOLUTION, yMagC * MAG_RESOLUTION, data_mag.dataz * MAG_RESOLUTION);
}

void BMX055_updateIMU(void)
{
	/* structure used for read the sensor data - xyz*/
	struct bmg160_data_t data_gyro;
	/* bma2x2acc_data structure used to read accel xyz data*/
	struct bma2x2_accel_data data_acc;

	bmg160_get_data_XYZ(&data_gyro);
	bma2x2_read_accel_xyz(&data_acc);
	
	MahonyAHRSupdateIMU(data_gyro.datax * DEG2RAD * GYRO_RESOLUTION, data_gyro.datay * DEG2RAD * GYRO_RESOLUTION, data_gyro.dataz * DEG2RAD * GYRO_RESOLUTION,
	data_acc.x * ACC_RESOLUTION, data_acc.y * ACC_RESOLUTION, data_acc.z * ACC_RESOLUTION);	
}

float getPitch(float q0, float q1, float q2, float q3)
{
	float arg1 = 2.0f * (q1*q3 + q0*q2);
	float arg2 = sqrt(1.0f - pow((2.0f*q1*q3 + 2.0f*q0*q2), 2.0f));
  
	return -atan(arg1/arg2) * RAD2DEG;
}

float getYaw(float q0, float q1, float q2, float q3) 
{
	float arg1 = 2.0f*(q1*q2-q0*q3);
	float arg2 = 2.0f*q0*q0 - 1.0f + 2.0f*q1*q1;
  
	return atan2(arg1,arg2)*RAD2DEG;
}

float getRoll(float q0, float q1, float q2, float q3)
{
	float arg1 = 2.0f*(q2*q3-q0*q1);
	float arg2 = 2.0f*q0*q0 - 1.0f + 2.0f*q3*q3;
  
	return atan2(arg1,arg2)*RAD2DEG;
}




