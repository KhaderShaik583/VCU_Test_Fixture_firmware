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

/*!
 * File: msdi_s32k1xx.h
 *
 * This file implements external functions required by MSDI SW driver.
 * It is closely related to this demo example.
 */

#ifndef MSDI_S32K14X_H_
#define MSDI_S32K14X_H_
/*Include shared modules, which are used for whole project*/
#include "device_registers.h"
#include "board.h"
 
#include "clock_manager.h"
#include "interrupt_manager.h"
#include "edma_driver.h"
#include "osif.h"
#include "lpspi_master_driver.h"
#include "lpspi_slave_driver.h"
#include "lpspi_shared_function.h"
#include "adc_driver.h"
#include "lpuart_driver.h"
#include "lpit_driver.h"
#include "system_S32K148.h"
#include "pins_driver.h"
#include "msdi.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @addtogroup macro_group
 * @{
 */

/*! @brief LPSPI transfer timeout in milliseconds. If a transfer takes longer
 *  time, it is aborted and MSDI_STATUS_SPI_TIMEOUT error is reported. */
#define MSDI_SPI_TIMEOUT          100U

/* LPSPI configuration. */
#define MSDI_LPSPI_INSTANCE       0       /* LPSPI0 */
#define MSDI_LPSPI_BAUD           1000000 /* 1 MHz */
#define MSDI_LPSPI_SRCCLK         lpspiCom1_MasterConfig0.lpspiSrcClk
/* CSB - PTB5/LPSPI0_PCS1 */
#define MSDI_CSB_INSTANCE         SWIF_CS_GPIO
#define MSDI_CSB_PIN              SWIF_CS_PIN

/*! @} */

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @addtogroup function_group
 * @{
 */

/*!
 * @brief This function configures LPSPI for usage with MSDI driver.
 */
status_t MSDI_ConfigureLPSPI(void);

/*!
 * @brief This function performs one 32b LPSPI transfer to MSDI device.
 *
 * @param drvInstance Instance of MSDI driver.
 * @param sendData Data to be sent to MSDI device.
 * @param rcvData Pointer to memory where data received on MISO will be stored.
 */
msdi_status_t MSDI_TransferSpi(uint8_t drvInstance, const uint32_t sendData,
		uint32_t* const rcvData);

/*!
 * @brief User implementation of assert.
 *
 * @param x - True if everything is OK.
 */
void MSDI_Assert(bool x);

/*!
 * @brief Waits for specified amount of milliseconds.
 *
 * @param delay - Number of milliseconds to wait.
 */
void MSDI_WaitMs(uint16_t delay);

/*! @} */

#endif /* MSDI_S32K14X_H_ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
