/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * File: msdi_s32k1xx.c
 *
 * This file implements external functions required by MSDI SW driver.
 * It is closely related to this demo example.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "fw_common.h"
#include "msdi.h"
#include "msdi_s32k14x.h"
#include "drv_spi_legacy.h"

/*******************************************************************************
 * Global variables
 ******************************************************************************/

/* LPSPI state structure for each LPSPI instance. */
lpspi_state_t g_lpspiState[LPSPI_INSTANCE_COUNT];

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_ConfigureLPSPI
 * Description   : This function configures LPSPI for usage with BCC driver.
 *
 *END**************************************************************************/
/* In Board.c */

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_TransferSpi
 * Description   : This function performs one 32b LPSPI transfer to MSDI device.
 *
 *END**************************************************************************/
msdi_status_t MSDI_TransferSpi(uint8_t drvInstance, const uint32_t sendData,
		uint32_t* const rcvData)
{
    status_t status = STATUS_SUCCESS;
    uint8_t sendBuffer[4];
    uint8_t rcvBuffer[4];


    DEV_ASSERT(rcvData != NULL);
 
    sendBuffer[0] = (uint8_t)(sendData & 0xFFU);
    sendBuffer[1] = (uint8_t)((sendData >> 8) & 0xFFU);
    sendBuffer[2] = (uint8_t)((sendData >> 16) & 0xFFU);
    sendBuffer[3] = (uint8_t)((sendData >> 24) & 0xFFU); 
    
    /* Toggle CSB_M. */
    PINS_DRV_ClearPins(MSDI_CSB_INSTANCE, 1U << MSDI_CSB_PIN);

    status = LPSPI_DRV_MasterTransferBlocking(MSDI_LPSPI_INSTANCE, sendBuffer,
            rcvBuffer, 4U, MSDI_SPI_TIMEOUT);
    
    if (status == STATUS_TIMEOUT)
    {
        PINS_DRV_SetPins(MSDI_CSB_INSTANCE, 1U << MSDI_CSB_PIN);
        return MSDI_STATUS_SPI_TIMEOUT;
    }
    if (status != STATUS_SUCCESS)
    {
        PINS_DRV_SetPins(MSDI_CSB_INSTANCE, 1U << MSDI_CSB_PIN);
        return MSDI_STATUS_SPI_BUSY;
    }

    /* Toggle CSB_M. */
    PINS_DRV_SetPins(MSDI_CSB_INSTANCE, 1U << MSDI_CSB_PIN);

	*rcvData = ((uint32_t)rcvBuffer[0]) |
			(((uint32_t)rcvBuffer[1]) << 8U) |
			(((uint32_t)rcvBuffer[2]) << 16U) |
			(((uint32_t)rcvBuffer[3]) << 24U);

    
    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_Assert
 * Description   : User implementation of assert.
 *
 *END**************************************************************************/
void MSDI_Assert(bool x)
{
    DEV_ASSERT(x);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_WaitMs
 * Description   : Waits for specified amount of milliseconds.
 *
 *END**************************************************************************/
void MSDI_WaitMs(uint16_t delay)
{
	sw_asm_delay_us(delay * 1000U);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
