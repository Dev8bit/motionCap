/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "bmx055_support.h"
#include "MahonyAHRS.h"

/*lint -save -e689 */ /* Apparent end of comment ignored */
#include "arm_const_structs.h"
/*lint -restore */

#define FPU_EXCEPTION_MASK               0x0000009F                      //!< FPU exception mask used to clear exceptions in FPSCR register.
#define FPU_FPSCR_REG_STACK_OFF          0x40                            //!< Offset of FPSCR register stacked during interrupt handling in FPU part stack.

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

#define PACKET_LENGTH   	(23)		//packet length of eMPL-pythonclient
#define PACKET_DEBUG    	(1)
#define PACKET_QUAT     	(2)
#define PACKET_DATA     	(3)


/* TWI instance. */
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

void eMPL_send_data(int32_t *quat)
{
    char out[PACKET_LENGTH];
	uint8_t uCount = 0;
    if (!quat)
        return;
    memset(out, 0, PACKET_LENGTH);
	//q31 to q30
	quat[0] = quat[0] >> 1;
	quat[1] = quat[1] >> 1;
	quat[2] = quat[2] >> 1;
	quat[3] = quat[3] >> 1;
	//convert long to char
    out[0] = '$';
    out[1] = PACKET_QUAT;
    out[3] = (char)(quat[0] >> 24);
    out[4] = (char)(quat[0] >> 16);
    out[5] = (char)(quat[0] >> 8);
    out[6] = (char)quat[0];
    out[7] = (char)(quat[1] >> 24);
    out[8] = (char)(quat[1] >> 16);
    out[9] = (char)(quat[1] >> 8);
    out[10] = (char)quat[1];
    out[11] = (char)(quat[2] >> 24);
    out[12] = (char)(quat[2] >> 16);
    out[13] = (char)(quat[2] >> 8);
    out[14] = (char)quat[2];
    out[15] = (char)(quat[3] >> 24);
    out[16] = (char)(quat[3] >> 16);
    out[17] = (char)(quat[3] >> 8);
    out[18] = (char)quat[3];
    out[21] = '\r';
    out[22] = '\n';

	while(uCount < PACKET_LENGTH)
	{
		NRF_LOG_RAW_INFO("%c", out[uCount++]);
	}
	
}

__STATIC_INLINE void data_handler(void)
{
	int32_t qData[4] = {0};
	float32_t fData[4] ={q0, q1, q2, q3};
	arm_float_to_q31(fData, qData, 4);
	
	eMPL_send_data(qData);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {

            }
            m_xfer_done = true;
            break;
		case NRF_DRV_TWI_EVT_ADDRESS_NACK:
			NRF_LOG_ERROR("\r\nNACK received after sending the address.");
			break;
		case NRF_DRV_TWI_EVT_DATA_NACK:
			NRF_LOG_ERROR("\r\nNACK received after sending a data byte.");
			break;
        default:
            break;
    }
}

/*You must clear exception flags in the FPSCR register and 
clear the pending FPU interrupt. Otherwise, MCU cannot go to sleep mode.*/
// Function handles and clears exception flags in FPSCR register and at the stack.
// During interrupt, handler execution FPU registers might be copied to the stack
// (see lazy stacking option) and it is necessary to clear data at the stack
// which will be recovered in the return from interrupt handling.
#ifdef FPU_INTERRUPT_MODE
/**
 * @brief FPU Interrupt handler. Clearing exception flag at the stack.
 *
 * Function clears exception flag in FPSCR register and at the stack. During interrupt handler
 * execution FPU registers might be copied to the stack (see lazy stacking option) and
 * it is necessary to clear data at the stack which will be recovered in the return from
 * interrupt handling.
 */
void FPU_IRQHandler(void)
{
    // Prepare pointer to stack address with pushed FPSCR register.
    uint32_t * fpscr = (uint32_t * )(FPU->FPCAR + FPU_FPSCR_REG_STACK_OFF);
    // Execute FPU instruction to activate lazy stacking.
    (void)__get_FPSCR();
    // Clear flags in stacked FPSCR register.
    *fpscr = *fpscr & ~(FPU_EXCEPTION_MASK);
}
#endif


/**
 * @brief TWIM initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_BMX055_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = true
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_BMX055_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Function for reading data from sensor.
 */
static void read_sensor_data()
{
	static uint8_t magRstCount = 0;
	
	if(magRstCount++ % 4 == 0)
	{
		BMX055_updateAHRS();		
	}
	else
	{
		BMX055_updateIMU();		
	}
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{	
	int8_t com_rslt = 0;
	
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example started.");
    NRF_LOG_FLUSH();
	
	//init nrf52832 ICC peripheral
	twi_init();
	//config BMX055
    APP_ERROR_CHECK(BMX055_config());

	
#ifdef FPU_INTERRUPT_MODE
    // Enable FPU interrupt
    NVIC_SetPriority(FPU_IRQn, APP_IRQ_PRIORITY_LOWEST);
    NVIC_ClearPendingIRQ(FPU_IRQn);
    NVIC_EnableIRQ(FPU_IRQn);
#endif
	
	//BMX055_CalibrateMag
	com_rslt = BMX055_CalibrateMag(0);
	while(com_rslt)
	{
		NRF_LOG_INFO("\r\nBMX055_CalibrateMag failure!");
		nrf_delay_ms(1000);
	}
	
	
    while (true)
    {
        nrf_delay_ms(10);

        read_sensor_data();
		data_handler();
		
        NRF_LOG_FLUSH();
    }
}

/** @} */
