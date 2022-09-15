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
 * @file msdi.c
 *
 * MSDI driver supporting boards based on following NXP parts: CD1020, CD1030,
 * MC33978 and MC34978.
 *
 * This module is common for all supported models.
 *
 * Note that from the perspective of this driver, MC33978 and MC34978 devices
 * are similar. Therefore, comments targeting MC33978 device are intended for 
 * both MC33978 and MC34978 devices.
 */

#include "msdi.h"
#include "msdi_s32k14x.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Returns true if the MSDI SPI response to read/write operation
 * on selected register contains INTflg bit.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param reg Register from msdi_register_t enumeration.
 */
#define MSDI_HAS_INTFLG(drvConfig, reg) ( \
        (reg != MSDI_SPI_CHECK) && \
        (reg != MSDI_WET_CUR_SP_0) && \
        (reg != MSDI_WET_CUR_SG_0) && \
        ((!MSDI_IS_CD1030(drvConfig)) || (reg != MSDI_WET_CUR_SG_1)) && \
        (reg != MSDI_RESET))

/*!
 * @brief Returns true if the MSDI SPI response to read/write operation
 * on selected register contains FAULT STATUS bit.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param reg Register from msdi_register_t enumeration.
 */
#define MSDI_HAS_FS(drvConfig, reg) ( \
        (reg != MSDI_SPI_CHECK) && \
        (reg != MSDI_WET_CUR_SP_0) && \
        (reg != MSDI_WET_CUR_SG_0) && \
        ((!MSDI_IS_CD1030(drvConfig)) || (reg != MSDI_WET_CUR_SG_1)) && \
        (reg != MSDI_FLT_STAT) && \
        (reg != MSDI_RESET))

/*!
 * @brief Parses the register (from msdi_register_t enumeration) from the
 * LPSPI data frame.
 *
 * @param x LPSPI data frame.
 */
#define MSDI_GET_REG_NAME(x) \
    ((msdi_register_t)(((x) & MSDI_REG_ADDR_MASK) >> MSDI_REG_ADDR_SHIFT))

/*!
 * @brief Returns true CD1020 device is selected in the driver configuration.
 *
 * @param drvConfig Pointer to driver instance configuration.
 */
#define MSDI_IS_CD1020(drvConfig) (drvConfig->deviceType == MSDI_DEVICE_CD1020)

/*!
 * @brief Returns true CD1030 device is selected in the driver configuration.
 *
 * @param drvConfig Pointer to driver instance configuration.
 */
#define MSDI_IS_CD1030(drvConfig) (drvConfig->deviceType == MSDI_DEVICE_CD1030)

/*!
 * @brief Returns true MC33978 device is selected in the driver configuration.
 *
 * @param drvConfig Pointer to driver instance configuration.
 */
#define MSDI_IS_MC33978(drvConfig) (drvConfig->deviceType == MSDI_DEVICE_MC33978)

/*!
 * @brief Number of SP pins for a specific MSDI device.
 *
 * @param drvConfig Pointer to driver instance configuration.
 */
#define MSDI_SP_CNT(drvConfig)         \
    (MSDI_IS_CD1030(drvConfig) ? MSDI_SP_CNT_CD1030 : MSDI_SP_CNT_CD1020)

/*!
 * @brief Number of SG pins for a specific MSDI device.
 *
 * @param drvConfig Pointer to driver instance configuration.
 */
#define MSDI_SG_CNT(drvConfig)         \
    (MSDI_IS_CD1030(drvConfig) ? MSDI_SG_CNT_CD1030 : MSDI_SG_CNT_CD1020)

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*!
 * @brief This function performs one 32b LPSPI transfer and stores data received
 * on MISO to rcvData. It is based on user implementation of MSDI_TransferSpi.
 * INTflg and FAULT STATUS bits are parsed from the received data and stored to
 * the driver configuration structure.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sendData Data to be sent on MOSI.
 * @param rcvData Pointer to memory where data received on MISO is stored. Use
 *                NULL if no data should be stored.
 */
static msdi_status_t MSDI_TransferSpiAndParse(msdi_drv_config_t* const drvConfig,
        const uint32_t sendData, uint32_t* const rcvData);

/*!
 * @brief This function serves to reading and writing a MSDI register via LPSPI.
 * It performs two 32b LPSPI transfers, where the second one serves to check the
 * first one (in case of write operation) or to receive the register data (in
 * case of read operation).
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param regName Register from msdi_register_t enumeration.
 * @param sendData Pointer to a 32b memory with data sent to MSDI in the first
 *                 MSDI transfer. Use NULL in case of register read operation.
 * @param rcvData Pointer to a 32b memory where register content is stored. Use
 *                NULL if you do not need the data part of second SPI transfer.
 */
static msdi_status_t MSDI_DataTransfer(msdi_drv_config_t* const drvConfig,
        const msdi_register_t regName, const uint32_t* const sendData,
        uint32_t* const receiveData);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_TransferSpiAndParse
 * Description   : This function performs one 32b LPSPI transfer and stores data
 *                 received on MISO. INTflg and FAULT STATUS bits are parsed
 *                 from the received data and stored to the driver configuration
 *                 structure.
 *
 *END**************************************************************************/
static msdi_status_t MSDI_TransferSpiAndParse(msdi_drv_config_t* const drvConfig,
        const uint32_t sendData, uint32_t* const rcvData)
{
    msdi_status_t status = MSDI_STATUS_SUCCESS;
    uint32_t receiveData;

    MSDI_Assert(drvConfig != NULL);

    status = MSDI_TransferSpi(drvConfig->drvInstance, sendData, &receiveData);

    /* Parse the FAULT STATUS and INTflg flags from the transfers. */
    if (MSDI_HAS_FS(drvConfig, MSDI_GET_REG_NAME(receiveData)))
    {
        drvConfig->faultStatus = (receiveData & MSDI_FAULT_STATUS_MASK) ? true : false;
    }

    if (MSDI_HAS_INTFLG(drvConfig, MSDI_GET_REG_NAME(receiveData)))
    {
        drvConfig->intFlg = (receiveData & MSDI_INT_FLG_MASK) ? true : drvConfig->intFlg;
    }

    /* Fill the receiveData parameter. */
    if (rcvData != NULL)
    {
        *rcvData = receiveData;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_DataTransfer
 * Description   : This function serves to reading and writing a MSDI register
 *                 via LPSPI.
 *
 *END**************************************************************************/
static msdi_status_t MSDI_DataTransfer(msdi_drv_config_t* const drvConfig,
        const msdi_register_t regName, const uint32_t* const sendData,
        uint32_t* const receiveData)
{
    msdi_status_t status = MSDI_STATUS_SUCCESS;
    uint32_t cmd = 0U;
    uint32_t readData = 0U;

    MSDI_Assert(drvConfig != NULL);

    /* Test if there is data for sending. */
    if (sendData == NULL)
    {
        cmd = MSDI_REG_ADDR_F(regName) | MSDI_REG_RW_R;
    }
    else
    {
        cmd = MSDI_REG_ADDR_F(regName) | MSDI_REG_RW_W | MSDI_REG_DATA_F(*sendData);
    }

    /* Transfer of first SPI word. */
    if ((status = MSDI_TransferSpiAndParse(drvConfig, cmd, NULL)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    /* Transfer of second SPI word. */
    if ((status = MSDI_TransferSpiAndParse(drvConfig, 0U, &readData)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    /* Check if the frame received in the second SPI transfer correspond to the
     * frame sent in the first SPI transfer. */
    if (sendData == NULL)
    {
        /* Read transfer */
        if ((readData & (MSDI_REG_ADDR_MASK | MSDI_REG_RW_MASK)) != (cmd & (MSDI_REG_ADDR_MASK | MSDI_REG_RW_MASK)))
        {
            /* Register name or R/W bit does not correspond. */
            return MSDI_STATUS_COMM_ERROR;
        }
    }
    else if (MSDI_HAS_FS(drvConfig, regName) || MSDI_HAS_INTFLG(drvConfig, regName))
    {
        /* Write transfer, where received frame contains STATUS_FLAG or INTflg. */
        if ((readData & MSDI_RED_RW_DATA_MASK) != (cmd & MSDI_RED_RW_DATA_MASK))
        {
            /* Register name, R/W bit or register data does not correspond. */
            return MSDI_STATUS_COMM_ERROR;
        }
    }
    else if (readData != cmd)
    {
        /* Write transfer, where sent and received frame should be equal but
         * they are not. */
        return MSDI_STATUS_COMM_ERROR;
    }

    /* Copy content of the register to receive data. */
    if (receiveData != NULL)
    {
        *receiveData = readData & MSDI_REG_DATA_MASK;
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetDefaultInitConfig
 * Description   : This function fills msdi_init_config_t structure with default
 *                 MSDI device data after POR.
 *
 *END**************************************************************************/
void MSDI_GetDefaultInitConfig(const msdi_drv_config_t* const drvConfig,
        msdi_init_config_t* const initConfig)
{
    uint8_t i;

    MSDI_Assert(drvConfig != NULL);
    MSDI_Assert(initConfig != NULL);

    initConfig->devConfig.sbpolltime = MSDI_SBPOLLTIME_1000;
    initConfig->devConfig.vbatOvDis = MSDI_VBAT_OV_EN;
    initConfig->devConfig.wakeVddqCheck = MSDI_WAKE_B_NO_CHECK;
    initConfig->devConfig.intBehavior = MSDI_INT_LOW;
    initConfig->devConfig.amuxCtrl = MSDI_AMUX_CTRL_SPI_DEF;
    initConfig->devConfig.spx = MSDI_IS_CD1030(drvConfig) ? MSDI_CFG_SPX_DEF_CD1030 : MSDI_CFG_SPX_DEF_CD1020;
    initConfig->triStateSp = MSDI_IS_CD1030(drvConfig) ? MSDI_TRI_STATE_SP_DEF_CD1030 : MSDI_TRI_STATE_SP_DEF_CD1020;
    initConfig->triStateSg = MSDI_IS_CD1030(drvConfig) ? MSDI_TRI_STATE_SG_DEF_CD1030 : MSDI_TRI_STATE_SG_DEF_CD1020;

    for (i = 0; i < MSDI_SP_CNT(drvConfig); i++)
    {
        initConfig->wetCurLevelSp[i] = MSDI_WET_CUR_2;
    }

    for (i = 0; i < MSDI_SG_CNT(drvConfig); i++)
    {
        initConfig->wetCurLevelSg[i] = MSDI_WET_CUR_2;
    }

    initConfig->contWetCurEnSp = MSDI_CONT_WET_SP_DEF;
    initConfig->contWetCurEnSg = MSDI_CONT_WET_SG_DEF;
    initConfig->intEnSp = MSDI_IS_CD1030(drvConfig) ? MSDI_INT_EN_SP_DEF_CD1030 : MSDI_INT_EN_SP_DEF_CD1020;
    initConfig->intEnSg = MSDI_IS_CD1030(drvConfig) ? MSDI_INT_EN_SG_DEF_CD1030 : MSDI_INT_EN_SG_DEF_CD1020;
    initConfig->lpmConfig.intTmrVal = MSDI_INT_TMR_VAL_OFF;
    initConfig->lpmConfig.pollRate = MSDI_POLL_RATE_64;
    initConfig->wakeUpEnSp = MSDI_IS_CD1030(drvConfig) ? MSDI_WAKE_EN_SP_DEF_CD1030 : MSDI_WAKE_EN_SP_DEF_CD1020;
    initConfig->wakeUpEnSg = MSDI_IS_CD1030(drvConfig) ? MSDI_WAKE_EN_SG_DEF_CD1030 : MSDI_WAKE_EN_SG_DEF_CD1020;
    initConfig->lpmCompOnlySp = MSDI_COMP_ONLY_SP_DEF;
    initConfig->lpmCompOnlySg = MSDI_COMP_ONLY_SG_DEF;
    initConfig->lpmVoltThresholdSp = MSDI_LPM_THR_SP_DEF;
    initConfig->lpmVoltThresholdSg = MSDI_LPM_THR_SG_DEF;
    initConfig->lpmPollCurConfigSp = MSDI_POLL_CUR_SP_DEF;
    initConfig->lpmPollCurConfigSg = MSDI_POLL_CUR_SG_DEF;
    initConfig->slowPollingSp = MSDI_SLOW_POLL_SP_DEF;
    initConfig->slowPollingSg = MSDI_SLOW_POLL_SG_DEF;
    initConfig->wakeUpDebounceSp = MSDI_WAKE_DEB_SP_DEF;
    initConfig->wakeUpDebounceSg = MSDI_WAKE_DEB_SG_DEF;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_Init
 * Description   : This function checks SPI communication with the MSDI device
 *                 and initializes all SPI configurable registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_Init(msdi_drv_config_t* const drvConfig,
        const msdi_init_config_t* const initConfig)
{
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    drvConfig->intFlg = 0;
    drvConfig->faultStatus = 0;

    /* Check the SPI communication */
    if ((status = MSDI_CheckSpi(drvConfig)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    /* Set the registers. */
    if (initConfig != NULL)
    {
        if ((status = MSDI_SetDeviceConfig(drvConfig, &(initConfig->devConfig))) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        if ((status = MSDI_SetWetCurLevel(drvConfig, initConfig->wetCurLevelSp, initConfig->wetCurLevelSg)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        if ((status = MSDI_SetContWetCurEnable(drvConfig, initConfig->contWetCurEnSp, initConfig->contWetCurEnSg)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        if ((status = MSDI_SetIntEnable(drvConfig, initConfig->intEnSp, initConfig->intEnSg)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        if ((status = MSDI_SetLPMConfig(drvConfig, &(initConfig->lpmConfig))) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        if ((status = MSDI_SetWakeUpEnable(drvConfig, initConfig->wakeUpEnSp, initConfig->wakeUpEnSg)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        if ((status = MSDI_SetComparatorOnly(drvConfig, initConfig->lpmCompOnlySp, initConfig->lpmCompOnlySg)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        if ((status = MSDI_SetLPMVoltThreshold(drvConfig, initConfig->lpmVoltThresholdSp, initConfig->lpmVoltThresholdSg)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        if ((status = MSDI_SetLPMPollCurrent(drvConfig, initConfig->lpmPollCurConfigSp, initConfig->lpmPollCurConfigSg)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        if (!MSDI_IS_CD1020(drvConfig))
        {
            if ((status = MSDI_SetSlowPolling(drvConfig, initConfig->slowPollingSp, initConfig->slowPollingSg)) != MSDI_STATUS_SUCCESS)
            {
                return status;
            }

            if ((status = MSDI_SetWakeUpDebounce(drvConfig, initConfig->wakeUpDebounceSp, initConfig->wakeUpDebounceSg)) != MSDI_STATUS_SUCCESS)
            {
                return status;
            }
        }

        if ((status = MSDI_SetTriStateEnable(drvConfig, initConfig->triStateSp, initConfig->triStateSg)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_CheckSpi
 * Description   : This function checks the SPI communication using Check SPI
 *                 command.
 *
 *END**************************************************************************/
msdi_status_t MSDI_CheckSpi(msdi_drv_config_t* const drvConfig)
{
    uint32_t rcvData;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_SPI_CHECK, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    /* Check the answer. */
    if (rcvData != MSDI_SPI_CHECK_ASW)
    {
        return MSDI_STATUS_COMM_ERROR;
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_SetDeviceConfig
 * Description   : This function sets the Device Configuration register.
 *
 *END**************************************************************************/
msdi_status_t MSDI_SetDeviceConfig(msdi_drv_config_t* const drvConfig,
        const msdi_dev_cfg_t* const cfg)
{
    uint32_t regVal = 0U;

    MSDI_Assert(drvConfig != NULL);
    MSDI_Assert(cfg != NULL);

    if (MSDI_IS_CD1030(drvConfig))
    {
        regVal = MSDI_CFG_SBPOLLTIME_F_CD1030(cfg->sbpolltime);
        regVal |= MSDI_CFG_VBATP_OV_F_CD1030(cfg->vbatOvDis);
        regVal |= MSDI_CFG_WAKE_B_F_CD1030(cfg->wakeVddqCheck);
        regVal |= MSDI_CFG_INT_B_OUT_F_CD1030(cfg->intBehavior);
        regVal |= MSDI_CFG_ACONFIG_F_CD1030(cfg->amuxCtrl);
        regVal |= MSDI_CFG_SPX_F_CD1030(cfg->spx);
    }
    else
    {
        regVal = MSDI_CFG_SBPOLLTIME_F_CD1020(cfg->sbpolltime);
        regVal |= MSDI_CFG_VBATP_OV_F_CD1020(cfg->vbatOvDis);
        regVal |= MSDI_CFG_WAKE_B_F_CD1020(cfg->wakeVddqCheck);
        regVal |= MSDI_CFG_INT_B_OUT_F_CD1020(cfg->intBehavior);
        regVal |= MSDI_CFG_SPX_F_CD1020(cfg->spx);

        if (MSDI_IS_MC33978(drvConfig))
        {
            regVal |= MSDI_CFG_ACONFIG_F_MC33978(cfg->amuxCtrl);
        }
    }

    return MSDI_DataTransfer(drvConfig, MSDI_DEV_CFG, &regVal, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetDeviceConfig
 * Description   : This function reads the Device Configuration register.
 *
 *END**************************************************************************/
msdi_status_t MSDI_GetDeviceConfig(msdi_drv_config_t* const drvConfig,
        msdi_dev_cfg_t* const cfg)
{
    uint32_t regVal = 0U;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);
    MSDI_Assert(cfg != NULL);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_DEV_CFG, NULL, &regVal)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    if (MSDI_IS_CD1030(drvConfig))
    {
        cfg->sbpolltime = MSDI_CFG_SBPOLLTIME_P_CD1030(regVal);
        cfg->vbatOvDis = MSDI_CFG_VBATP_OV_P_CD1030(regVal);
        cfg->wakeVddqCheck = MSDI_CFG_WAKE_B_P_CD1030(regVal);
        cfg->intBehavior = MSDI_CFG_INT_B_OUT_P_CD1030(regVal);
        cfg->amuxCtrl = MSDI_CFG_ACONFIG_P_CD1030(regVal);
        cfg->spx = MSDI_CFG_SPX_P_CD1030(regVal);
    }
    else
    {
        cfg->sbpolltime = MSDI_CFG_SBPOLLTIME_P_CD1020(regVal);
        cfg->vbatOvDis = MSDI_CFG_VBATP_OV_P_CD1020(regVal);
        cfg->wakeVddqCheck = MSDI_CFG_WAKE_B_P_CD1020(regVal);
        cfg->intBehavior = MSDI_CFG_INT_B_OUT_P_CD1020(regVal);
        cfg->spx = MSDI_CFG_SPX_P_CD1020(regVal);

        if (MSDI_IS_MC33978(drvConfig))
        {
            cfg->amuxCtrl = MSDI_CFG_ACONFIG_P_MC33978(regVal);
        }
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_SetTriStateEnable
 * Description   : This function sets the Tri-state registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_SetTriStateEnable(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg)
{
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    sp &= (MSDI_IS_CD1030(drvConfig) ? MSDI_TRI_STATE_SP_MASK_CD1030 : MSDI_TRI_STATE_SP_MASK_CD1020);
    sg &= (MSDI_IS_CD1030(drvConfig) ? MSDI_TRI_STATE_SG_MASK_CD1030 : MSDI_TRI_STATE_SG_MASK_CD1020);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_TRI_STATE_SP, &sp, NULL)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    return MSDI_DataTransfer(drvConfig, MSDI_TRI_STATE_SG, &sg, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetTriStateEnable
 * Description   : This function reads the Tri-state registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_GetTriStateEnable(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg)
{
    uint32_t rcvData;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    if (sp != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_TRI_STATE_SP, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sp = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_TRI_STATE_SP_MASK_CD1030 : MSDI_TRI_STATE_SP_MASK_CD1020);
    }

    if (sg != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_TRI_STATE_SG, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sg = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_TRI_STATE_SG_MASK_CD1030 : MSDI_TRI_STATE_SG_MASK_CD1020);
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_SetWetCurLevel
 * Description   : This function sets the Wetting Current Level registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_SetWetCurLevel(msdi_drv_config_t* const drvConfig,
        const msdi_wet_cur_t* const sp, const msdi_wet_cur_t* const sg)
{
    uint8_t i;
    uint32_t regValue;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    /* Check the wetting current levels for CD1020 (value must be even). */
    if (sp != NULL)
    {
        for (i = 0; i < MSDI_SP_CNT_CD1020; i++)
        {
            if ((((uint8_t)(sp[i])) & 0x01U) != 0x00U)
            {
                return MSDI_STATUS_PARAM_RANGE;
            }
        }
    }
    if (sg != NULL)
    {
        for (i = 0; i < MSDI_SG_CNT_CD1020; i++)
        {
            if ((((uint8_t)(sg[i])) & 0x01U) != 0x00U)
            {
                return MSDI_STATUS_PARAM_RANGE;
            }
        }
    }

    /* Write to registers. */
    if (sp != NULL)
    {
        /* Set Wetting current level SP register 0 (SP0 - SP7). */
        regValue = 0;
        for (i = MSDI_WET_CUR_SP_0_SP_MIN; i <= MSDI_WET_CUR_SP_0_SP_MAX; i++)
        {
            regValue |= MSDI_WET_CUR_SP_0_F(sp[i], i);
        }
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_WET_CUR_SP_0, &regValue, NULL)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        /* Set Wetting current level SP register 1 (SP8 - SP11). */
        if (MSDI_IS_CD1030(drvConfig))
        {
            regValue = 0;
            for (i = MSDI_WET_CUR_SP_1_SP_MIN; i <= MSDI_WET_CUR_SP_1_SP_MAX; i++)
            {
                regValue |= MSDI_WET_CUR_SP_1_F_CD1030(sp[i], i);
            }
            if ((status = MSDI_DataTransfer(drvConfig, MSDI_WET_CUR_SP_1, &regValue, NULL)) != MSDI_STATUS_SUCCESS)
            {
                return status;
            }
        }
    }

    if (sg != NULL)
    {
        /* Set Wetting current level SG register 0 (SG0 - SG7). */
        regValue = 0;
        for (i = MSDI_WET_CUR_SG_0_SG_MIN; i <= MSDI_WET_CUR_SG_0_SG_MAX; i++)
        {
            regValue |= MSDI_WET_CUR_SG_0_F(sg[i], i);
        }
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_WET_CUR_SG_0, &regValue, NULL)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        /* Set Wetting current level SG register 1 (SG8 - SG13/SG15). */
        regValue = 0;
        for (i = MSDI_WET_CUR_SG_1_SG_MIN; i <= (MSDI_IS_CD1030(drvConfig) ? MSDI_WET_CUR_SG_1_SG_MAX_CD1030 : MSDI_WET_CUR_SG_1_SG_MAX_CD1020); i++)
        {
            regValue |= MSDI_WET_CUR_SG_1_F(sg[i], i);
        }
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_WET_CUR_SG_1, &regValue, NULL)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        /* Set Wetting current level SG register 2 (SG16 - SG20). */
        if (MSDI_IS_CD1030(drvConfig))
        {
            regValue = 0;
            for (i = MSDI_WET_CUR_SG_2_SG_MIN; i <= MSDI_WET_CUR_SG_2_SG_MAX; i++)
            {
                regValue |= MSDI_WET_CUR_SG_2_F_CD1030(sg[i], i);
            }
            if ((status = MSDI_DataTransfer(drvConfig, MSDI_WET_CUR_SG_2, &regValue, NULL)) != MSDI_STATUS_SUCCESS)
            {
                return status;
            }
        }
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetWetCurLevel
 * Description   : This function reads the Wetting Current Level registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_GetWetCurLevel(msdi_drv_config_t* const drvConfig,
        msdi_wet_cur_t* const sp, msdi_wet_cur_t* const sg)
{
    uint8_t i;
    uint32_t rcvData;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    if (sp != NULL)
    {
        /* Read Wetting current level SP register 0 (SP0 - SP7). */
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_WET_CUR_SP_0, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }
        for (i = MSDI_WET_CUR_SP_0_SP_MIN; i <= MSDI_WET_CUR_SP_0_SP_MAX; i++)
        {
            sp[i] = MSDI_WET_CUR_SP_0_P(rcvData, i);
        }

        /* Read Wetting current level SP register 1 (SP8 - SP11). */
        if (MSDI_IS_CD1030(drvConfig))
        {
            if ((status = MSDI_DataTransfer(drvConfig, MSDI_WET_CUR_SP_1, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
            {
                return status;
            }
            for (i = MSDI_WET_CUR_SP_1_SP_MIN; i <= MSDI_WET_CUR_SP_1_SP_MAX; i++)
            {
                sp[i] = MSDI_WET_CUR_SP_1_P_CD1030(rcvData, i);
            }
        }
    }

    if (sg != NULL)
    {
        /* Read Wetting current level SG register 0 (SG0 - SG7). */
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_WET_CUR_SG_0, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }
        for (i = MSDI_WET_CUR_SG_0_SG_MIN; i <= MSDI_WET_CUR_SG_0_SG_MAX; i++)
        {
            sg[i] = MSDI_WET_CUR_SG_0_P(rcvData, i);
        }

        /* Read Wetting current level SG register 1 (SG8 - SG13/SG15). */
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_WET_CUR_SG_1, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }
        for (i = MSDI_WET_CUR_SG_1_SG_MIN; i <= (MSDI_IS_CD1030(drvConfig) ? MSDI_WET_CUR_SG_1_SG_MAX_CD1030 : MSDI_WET_CUR_SG_1_SG_MAX_CD1020); i++)
        {
            sg[i] = MSDI_WET_CUR_SG_1_P(rcvData, i);
        }

        /* Read Wetting current level SG register 2 (SG16 - SG20). */
        if (MSDI_IS_CD1030(drvConfig))
        {
            if ((status = MSDI_DataTransfer(drvConfig, MSDI_WET_CUR_SG_2, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
            {
                return status;
            }
            for (i = MSDI_WET_CUR_SG_2_SG_MIN; i <= MSDI_WET_CUR_SG_2_SG_MAX; i++)
            {
                sg[i] = MSDI_WET_CUR_SG_2_P_CD1030(rcvData, i);
            }
        }
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_SetContWetCurEnable
 * Description   : This function sets the Continuous wetting current registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_SetContWetCurEnable(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg)
{
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    sp &= (MSDI_IS_CD1030(drvConfig) ? MSDI_CONT_WET_SP_MASK_CD1030 : MSDI_CONT_WET_SP_MASK_CD1020);
    sg &= (MSDI_IS_CD1030(drvConfig) ? MSDI_CONT_WET_SG_MASK_CD1030 : MSDI_CONT_WET_SG_MASK_CD1020);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_CONT_WET_SP, &sp, NULL)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    return MSDI_DataTransfer(drvConfig, MSDI_CONT_WET_SG, &sg, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetContWetCurEnable
 * Description   : This function reads the Continuous wetting current registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_GetContWetCurEnable(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg)
{
    uint32_t rcvData;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    if (sp != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_CONT_WET_SP, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sp = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_CONT_WET_SP_MASK_CD1030 : MSDI_CONT_WET_SP_MASK_CD1020);
    }

    if (sg != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_CONT_WET_SG, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sg = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_CONT_WET_SG_MASK_CD1030 : MSDI_CONT_WET_SG_MASK_CD1020);
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_SetIntEnable
 * Description   : This function sets the Interrupt enable registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_SetIntEnable(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg)
{
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    sp &= (MSDI_IS_CD1030(drvConfig) ? MSDI_INT_EN_SP_MASK_CD1030 : MSDI_INT_EN_SP_MASK_CD1020);
    sg &= (MSDI_IS_CD1030(drvConfig) ? MSDI_INT_EN_SG_MASK_CD1030 : MSDI_INT_EN_SG_MASK_CD1020);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_INT_EN_SP, &sp, NULL)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    return MSDI_DataTransfer(drvConfig, MSDI_INT_EN_SG, &sg, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetIntEnable
 * Description   : This function reads the Interrupt enable registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_GetIntEnable(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg)
{
    uint32_t rcvData;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    if (sp != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_INT_EN_SP, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sp = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_INT_EN_SP_MASK_CD1030 : MSDI_INT_EN_SP_MASK_CD1020);
    }

    if (sg != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_INT_EN_SG, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sg = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_INT_EN_SG_MASK_CD1030 : MSDI_INT_EN_SG_MASK_CD1020);
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_SetLPMConfig
 * Description   : This function sets the Low-power mode configuration register.
 *
 *END**************************************************************************/
msdi_status_t MSDI_SetLPMConfig(msdi_drv_config_t* const drvConfig,
        const msdi_lpm_cfg_t* const lpmConfig)
{
    uint32_t regVal = 0U;

    MSDI_Assert(drvConfig != NULL);
    MSDI_Assert(lpmConfig != NULL);

    regVal = MSDI_LPM_CFG_POLL_F(lpmConfig->pollRate);

    if (!MSDI_IS_CD1020(drvConfig))
    {
        regVal |= MSDI_LPM_CFG_INT_F(lpmConfig->intTmrVal);
    }

    return MSDI_DataTransfer(drvConfig, MSDI_LPM_CFG, &regVal, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetLPMConfig
 * Description   : This function reads the Low-power mode configuration
 *                 register.
 *
 *END**************************************************************************/
msdi_status_t MSDI_GetLPMConfig(msdi_drv_config_t* const drvConfig,
        msdi_lpm_cfg_t* const lpmConfig)
{
    uint32_t regVal = 0U;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);
    MSDI_Assert(lpmConfig != NULL);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_LPM_CFG, NULL, &regVal)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    lpmConfig->pollRate = MSDI_LPM_CFG_POLL_P(regVal);

    if (!MSDI_IS_CD1020(drvConfig))
    {
        lpmConfig->intTmrVal = MSDI_LPM_CFG_INT_P(regVal);
    }

    return MSDI_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_SetWakeUpEnable
 * Description   : This function sets the Wake-up enable registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_SetWakeUpEnable(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg)
{
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    sp &= (MSDI_IS_CD1030(drvConfig) ? MSDI_WAKE_EN_SP_MASK_CD1030 : MSDI_WAKE_EN_SP_MASK_CD1020);
    sg &= (MSDI_IS_CD1030(drvConfig) ? MSDI_WAKE_EN_SG_MASK_CD1030 : MSDI_WAKE_EN_SG_MASK_CD1020);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_WAKE_EN_SP, &sp, NULL)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    return MSDI_DataTransfer(drvConfig, MSDI_WAKE_EN_SG, &sg, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetWakeUpEnable
 * Description   : This function reads the Wake-up enable registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_GetWakeUpEnable(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg)
{
    uint32_t rcvData;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    if (sp != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_WAKE_EN_SP, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sp = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_WAKE_EN_SP_MASK_CD1030 : MSDI_WAKE_EN_SP_MASK_CD1020);
    }

    if (sg != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_WAKE_EN_SG, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sg = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_WAKE_EN_SG_MASK_CD1030 : MSDI_WAKE_EN_SG_MASK_CD1020);
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_SetComparatorOnly
 * Description   : This function sets the Comparator-only registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_SetComparatorOnly(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg)
{
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    sp &= (MSDI_IS_CD1030(drvConfig) ? MSDI_COMP_ONLY_SP_MASK_CD1030 : MSDI_COMP_ONLY_SP_MASK_CD1020);
    sg &= (MSDI_IS_CD1030(drvConfig) ? MSDI_COMP_ONLY_SG_MASK_CD1030 : MSDI_COMP_ONLY_SG_MASK_CD1020);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_COMP_ONLY_SP, &sp, NULL)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    return MSDI_DataTransfer(drvConfig, MSDI_COMP_ONLY_SG, &sg, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetComparatorOnly
 * Description   : This function reads the Comparator-only registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_GetComparatorOnly(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg)
{
    uint32_t rcvData;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    if (sp != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_COMP_ONLY_SP, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sp = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_COMP_ONLY_SP_MASK_CD1030 : MSDI_COMP_ONLY_SP_MASK_CD1020);
    }

    if (sg != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_COMP_ONLY_SG, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sg = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_COMP_ONLY_SG_MASK_CD1030 : MSDI_COMP_ONLY_SG_MASK_CD1020);
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_SetLPMVoltThreshold
 * Description   : This function sets the LPM voltage threshold configuration
 *                 registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_SetLPMVoltThreshold(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg)
{
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    sp &= (MSDI_IS_CD1030(drvConfig) ? MSDI_LPM_THR_SP_MASK_CD1030 : MSDI_LPM_THR_SP_MASK_CD1020);
    sg &= (MSDI_IS_CD1030(drvConfig) ? MSDI_LPM_THR_SG_MASK_CD1030 : MSDI_LPM_THR_SG_MASK_CD1020);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_LPM_THR_SP, &sp, NULL)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    return MSDI_DataTransfer(drvConfig, MSDI_LPM_THR_SG, &sg, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetLPMVoltThreshold
 * Description   : This function reads the LPM voltage threshold configuration
 *                 registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_GetLPMVoltThreshold(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg)
{
    uint32_t rcvData;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    if (sp != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_LPM_THR_SP, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sp = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_LPM_THR_SP_MASK_CD1030 : MSDI_LPM_THR_SP_MASK_CD1020);
    }

    if (sg != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_LPM_THR_SG, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sg = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_LPM_THR_SG_MASK_CD1030 : MSDI_LPM_THR_SG_MASK_CD1020);
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_SetLPMPollCurrent
 * Description   : This function sets the Polling current configuration
 *                 registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_SetLPMPollCurrent(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg)
{
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    sp &= (MSDI_IS_CD1030(drvConfig) ? MSDI_POLL_CUR_SP_MASK_CD1030 : MSDI_POLL_CUR_SP_MASK_CD1020);
    sg &= (MSDI_IS_CD1030(drvConfig) ? MSDI_POLL_CUR_SG_MASK_CD1030 : MSDI_POLL_CUR_SG_MASK_CD1020);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_POLL_CUR_SP, &sp, NULL)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    return MSDI_DataTransfer(drvConfig, MSDI_POLL_CUR_SG, &sg, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetLPMPollCurrent
 * Description   : This function reads the Polling current configuration
 *                 registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_GetLPMPollCurrent(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg)
{
    uint32_t rcvData;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    if (sp != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_POLL_CUR_SP, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sp = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_POLL_CUR_SP_MASK_CD1030 : MSDI_POLL_CUR_SP_MASK_CD1020);
    }

    if (sg != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_POLL_CUR_SG, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sg = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_POLL_CUR_SG_MASK_CD1030 : MSDI_POLL_CUR_SG_MASK_CD1020);
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_SetSlowPolling
 * Description   : This function sets the Slow polling registers registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_SetSlowPolling(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg)
{
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);
    MSDI_Assert(MSDI_IS_CD1030(drvConfig) || MSDI_IS_MC33978(drvConfig));

    sp &= (MSDI_IS_CD1030(drvConfig) ? MSDI_SLOW_POLL_SP_MASK_CD1030 : MSDI_SLOW_POLL_SP_MASK_MC33978);
    sg &= (MSDI_IS_CD1030(drvConfig) ? MSDI_SLOW_POLL_SG_MASK_CD1030 : MSDI_SLOW_POLL_SG_MASK_MC33978);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_SLOW_POLL_SP, &sp, NULL)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    return MSDI_DataTransfer(drvConfig, MSDI_SLOW_POLL_SG, &sg, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetSlowPolling
 * Description   : This function reads the Slow polling registers registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_GetSlowPolling(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg)
{
    uint32_t rcvData;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);
    MSDI_Assert(MSDI_IS_CD1030(drvConfig) || MSDI_IS_MC33978(drvConfig));

    if (sp != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_SLOW_POLL_SP, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sp = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_SLOW_POLL_SP_MASK_CD1030 : MSDI_SLOW_POLL_SP_MASK_MC33978);
    }

    if (sg != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_SLOW_POLL_SG, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sg = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_SLOW_POLL_SG_MASK_CD1030 : MSDI_SLOW_POLL_SG_MASK_MC33978);
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_SetWakeUpDebounce
 * Description   : This function sets the Wake-up debounce registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_SetWakeUpDebounce(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg)
{
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);
    MSDI_Assert(MSDI_IS_CD1030(drvConfig) || MSDI_IS_MC33978(drvConfig));

    sp &= (MSDI_IS_CD1030(drvConfig) ? MSDI_WAKE_DEB_SP_MASK_CD1030 : MSDI_WAKE_DEB_SP_MASK_MC33978);
    sg &= (MSDI_IS_CD1030(drvConfig) ? MSDI_WAKE_DEB_SG_MASK_CD1030 : MSDI_WAKE_DEB_SG_MASK_MC33978);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_WAKE_DEB_SP, &sp, NULL)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    return MSDI_DataTransfer(drvConfig, MSDI_WAKE_DEB_SG, &sg, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetWakeUpDebounce
 * Description   : This function reads the Wake-up debounce registers.
 *
 *END**************************************************************************/
msdi_status_t MSDI_GetWakeUpDebounce(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg)
{
    uint32_t rcvData;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);
    MSDI_Assert(MSDI_IS_CD1030(drvConfig) || MSDI_IS_MC33978(drvConfig));

    if (sp != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_WAKE_DEB_SP, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sp = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_WAKE_DEB_SP_MASK_CD1030 : MSDI_WAKE_DEB_SP_MASK_MC33978);
    }

    if (sg != NULL)
    {
        if ((status = MSDI_DataTransfer(drvConfig, MSDI_WAKE_DEB_SG, NULL, &rcvData)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sg = rcvData & (MSDI_IS_CD1030(drvConfig) ? MSDI_WAKE_DEB_SG_MASK_CD1030 : MSDI_WAKE_DEB_SG_MASK_MC33978);
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_EnterLPM
 * Description   : This function switches MSDI device into Low-power mode.
 *
 *END**************************************************************************/
msdi_status_t MSDI_EnterLPM(msdi_drv_config_t* const drvConfig)
{
    MSDI_Assert(drvConfig != NULL);

    return MSDI_TransferSpiAndParse(drvConfig, MSDI_REG_ADDR_F(MSDI_ENTER_LPM) | MSDI_REG_RW_W, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_ExitLPM
 * Description   : This function interrupts MSDI Low-power mode by CSB pin and
 *                 waits t_CSB_WAKEUP time.
 *
 *END**************************************************************************/
msdi_status_t MSDI_ExitLPM(msdi_drv_config_t* const drvConfig)
{
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);

    status = MSDI_TransferSpiAndParse(drvConfig, MSDI_REG_ADDR_F(MSDI_SPI_CHECK), NULL);

    /* Wait the t_CSB_WAKEUP time (max. 1 ms). */
    MSDI_WaitMs(1);

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_SetAMUXChannel
 * Description   : This function sets the AMUX control register.
 *
 *END**************************************************************************/
msdi_status_t MSDI_SetAMUXChannel(msdi_drv_config_t* const drvConfig,
        const msdi_amux_cfg_t* const amuxCfg)
{
    uint32_t regVal = 0U;

    MSDI_Assert(drvConfig != NULL);
    MSDI_Assert(amuxCfg != NULL);

    regVal = MSDI_AMUX_CTRL_ASETT0_F(amuxCfg->current);
    regVal |= MSDI_AMUX_CTRL_ASEL_F(amuxCfg->channel);

    return MSDI_DataTransfer(drvConfig, MSDI_AMUX_CTRL, &regVal, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetAMUXChannel
 * Description   : This function reads the AMUX control register.
 *
 *END**************************************************************************/
msdi_status_t MSDI_GetAMUXChannel(msdi_drv_config_t* const drvConfig,
        msdi_amux_cfg_t* const amuxCfg)
{
    uint32_t regVal = 0U;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);
    MSDI_Assert(amuxCfg != NULL);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_AMUX_CTRL, NULL, &regVal)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    amuxCfg->current = MSDI_AMUX_CTRL_ASETT0_P(regVal);
    amuxCfg->channel = MSDI_AMUX_CTRL_ASEL_P(regVal);

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_ReadSwitchStatus
 * Description   : This function reads status of the switches.
 *
 *END**************************************************************************/
msdi_status_t MSDI_ReadSwitchStatus(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg)
{
    uint32_t regVal = 0U;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);
    MSDI_Assert(sp != NULL);
    MSDI_Assert(sg != NULL);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_READ_SW_STAT, NULL, &regVal)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }


    if (MSDI_IS_CD1030(drvConfig))
    {
        *sg = regVal & READ_SW_STAT_SG_MASK_CD1030;

        if ((status = MSDI_DataTransfer(drvConfig, MSDI_READ_SW_STAT_SP, NULL, &regVal)) != MSDI_STATUS_SUCCESS)
        {
            return status;
        }

        *sp = regVal & READ_SW_STAT_SP_MASK_CD1030;

    }
    else
    {
        *sg = READ_SW_STAT_SG_P_CD1020(regVal);
        *sp = READ_SW_STAT_SP_P_CD1020(regVal);
    }

    return MSDI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetFaultStatus
 * Description   : This function reads the Fault status register.
 *
 *END**************************************************************************/
msdi_status_t MSDI_GetFaultStatus(msdi_drv_config_t* const drvConfig,
        msdi_faults_t* const faultStatus)
{
    uint32_t regVal = 0U;
    msdi_status_t status = MSDI_STATUS_SUCCESS;

    MSDI_Assert(drvConfig != NULL);
    MSDI_Assert(faultStatus != NULL);

    if ((status = MSDI_DataTransfer(drvConfig, MSDI_FLT_STAT, NULL, &regVal)) != MSDI_STATUS_SUCCESS)
    {
        return status;
    }

    faultStatus->spiError = (regVal & MSDI_FLT_STAT_SPI_ERROR_MASK) > 0;
    faultStatus->hashFault = (regVal & MSDI_FLT_STAT_HASH_FLT_MASK) > 0;
    faultStatus->uv = (regVal & MSDI_FLT_STAT_UV_MASK) > 0;
    faultStatus->ov = (regVal & MSDI_FLT_STAT_OV_MASK) > 0;
    faultStatus->tempFlag = (regVal & MSDI_FLT_STAT_TEMP_FLG_MASK) > 0;
    faultStatus->ot = (regVal & MSDI_FLT_STAT_OV_MASK) > 0;
    faultStatus->intBWake = (regVal & MSDI_FLT_STAT_INT_WAKE_MASK) > 0;
    faultStatus->wakeBWake = (regVal & MSDI_FLT_STAT_WAKE_WAKE_MASK) > 0;
    faultStatus->spiWake = (regVal & MSDI_FLT_STAT_SPI_WAKE_MASK) > 0;
    faultStatus->por = (regVal & MSDI_FLT_STAT_POR_MASK) > 0;

    return MSDI_STATUS_SUCCESS;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_IntPulseRequest
 * Description   : This function requests MSDI device for an interrupt pulse.
 *
 *END**************************************************************************/
msdi_status_t MSDI_IntPulseRequest(msdi_drv_config_t* const drvConfig)
{
    uint32_t regVal = 0U;
    MSDI_Assert(drvConfig != NULL);

    return MSDI_DataTransfer(drvConfig, MSDI_INT_REQ, &regVal, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_Reset
 * Description   : This function causes all of the SPI registers to reset.
 *
 *END**************************************************************************/
msdi_status_t MSDI_Reset(msdi_drv_config_t* const drvConfig)
{
    MSDI_Assert(drvConfig != NULL);

    return MSDI_TransferSpiAndParse(drvConfig, MSDI_REG_ADDR_F(MSDI_RESET) | MSDI_REG_RW_W, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetFaultFlag
 * Description   : This function returns Fault status flag from the last SPI
 *                 transfer, which contained this flag.
 *
 *END**************************************************************************/
bool MSDI_GetFaultFlag(const msdi_drv_config_t* const drvConfig)
{
    MSDI_Assert(drvConfig != NULL);

    return drvConfig->faultStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MSDI_GetIntFlag
 * Description   : This function returns true if INTflg flag was set in any SPI
 *                 transfer from last calling of this function.
 *
 *END**************************************************************************/
bool MSDI_GetIntFlag(msdi_drv_config_t* const drvConfig)
{
    MSDI_Assert(drvConfig != NULL);

    bool intFlg = drvConfig->intFlg;
    drvConfig->intFlg = false;

    return intFlg;
}
