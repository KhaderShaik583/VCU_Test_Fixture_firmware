 /*
 * 
 * ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
 * ______________________________________
 * 
 * [2017] - [2018] Ultraviolette Automotive Pvt. Ltd.
 * All Rights Reserved.
 * 
 * NOTICE: All information contained herein is, and remains the 
 * property of Ultraviolette Automotive Pvt. Ltd. and its suppliers, if 
 * any. The intellectual and technical concepts contained herein are 
 * proprietary to Ultraviolette Automotive  and its suppliers and may 
 * be covered by U.S. and Foreign Patents, patents in process,  and 
 * are protected by trade secret or copyright law. Dissemination of 
 * this information or reproduction of this material is strictly 
 * forbidden unless prior written permission is obtained from 
 * Ultraviolette Automotive Pvt. Ltd.
 * 
 *
 * Author : Rishi F. [010]
 *


 */

#include "drv_i2c_legacy.h" 

static inline uint32_t LPI2C_MasterGetStatusFlags(LPI2C_Type *base)
{
    return base->MSR;
}

static inline void LPI2C_MasterClearStatusFlags(LPI2C_Type *base, uint32_t statusMask)
{
    base->MSR = statusMask;
}

static void LPI2C_MasterGetFifoCounts(LPI2C_Type *base, size_t *rxCount, size_t *txCount)
{
    if (txCount)
    {
        *txCount = (base->MFSR & LPI2C_MFSR_TXCOUNT_MASK) >> LPI2C_MFSR_TXCOUNT_SHIFT;
    }
    if (rxCount)
    {
        *rxCount = (base->MFSR & LPI2C_MFSR_RXCOUNT_MASK) >> LPI2C_MFSR_RXCOUNT_SHIFT;
    }
}

status_t LPI2C_MasterCheckAndClearError(LPI2C_Type *base, uint32_t status)
{
    status_t result = STATUS_SUCCESS;

    /* Check for error. These errors cause a stop to automatically be sent. We must */
    /* clear the errors before a new transfer can start. */
    status &= LPI2C_MSR_NDF_MASK|LPI2C_MSR_ALF_MASK|LPI2C_MSR_FEF_MASK|LPI2C_MSR_PLTF_MASK;
    if (status)
    {
        /* Select the correct error code. Ordered by severity, with bus issues first. */
        if (status & LPI2C_MSR_PLTF_MASK)
        {
            result = STATUS_ERROR;
        }
        else if (status & LPI2C_MSR_ALF_MASK)
        {
            result = STATUS_ERROR;
        }
        else if (status & LPI2C_MSR_NDF_MASK)
        {
            result = STATUS_ERROR;
        }
        else if (status & LPI2C_MSR_FEF_MASK)
        {
            result = STATUS_SUCCESS;
        }
        else
        {
            //assert(false);
        }

        /* Clear the flags. */
        LPI2C_MasterClearStatusFlags(base, status);

        /* Reset fifos. These flags clear automatically. */
        base->MCR |= LPI2C_MCR_RRF_MASK | LPI2C_MCR_RTF_MASK;
    }

    return result;
}

static status_t LPI2C_MasterWaitForTxReady(LPI2C_Type *base)
{
    uint32_t status;
    size_t txCount;
    size_t txFifoSize = 4U;
    do
    {
        status_t result;

        /* Get the number of words in the tx fifo and compute empty slots. */
        LPI2C_MasterGetFifoCounts(base, NULL, &txCount);
        txCount = txFifoSize - txCount;

        /* Check for error flags. */
        status = LPI2C_MasterGetStatusFlags(base);
        result = LPI2C_MasterCheckAndClearError(base, status);
        if (result)
        {
            return result;
        }
    } while (!txCount);

    return STATUS_SUCCESS;
}

status_t LPI2C_CheckForBusyBus(LPI2C_Type *base)
{
    uint32_t status = LPI2C_MasterGetStatusFlags(base);
    if ((status & LPI2C_MSR_BBF_MASK) && (!(status & LPI2C_MSR_MBF_MASK)))
    {
        return STATUS_BUSY;
    }

    return STATUS_SUCCESS;
}


status_t LPI2C_MasterStart(LPI2C_Type *base, uint8_t address, uint32_t dir)
{
    /* Return an error if the bus is already in use not by us. */
    status_t result = LPI2C_CheckForBusyBus(base);
    if (result)
    {
        return result;
    }

    /* Clear all flags. */
    LPI2C_MasterClearStatusFlags(base, LPI2C_MSR_EPF_MASK|LPI2C_MSR_SDF_MASK|LPI2C_MSR_NDF_MASK|LPI2C_MSR_ALF_MASK|LPI2C_MSR_FEF_MASK|LPI2C_MSR_PLTF_MASK|LPI2C_MSR_DMF_MASK);

    /* Turn off auto-stop option. */
    base->MCFGR1 &= ~LPI2C_MCFGR1_AUTOSTOP_MASK;

    /* Wait until there is room in the fifo. */
    result = LPI2C_MasterWaitForTxReady(base);
    if (result)
    {
        return result;
    }

    /* Issue start command. */
    base->MTDR = LPI2C_MTDR_CMD(0x4U) | (((uint32_t)address << 1U) | (uint32_t)dir);

    return STATUS_SUCCESS;
}

status_t LPI2C_MasterSend(LPI2C_Type *base, const void *txBuff, size_t txSize)
{
    const uint8_t *buf = (const uint8_t *)txBuff;

    //assert(txBuff);

    /* Send data buffer */
    while (txSize--)
    {
        /* Wait until there is room in the fifo. This also checks for errors. */
        status_t result = LPI2C_MasterWaitForTxReady(base);
        if (result)
        {
            return result;
        }

        /* Write byte into LPI2C master data register. */
        base->MTDR = *buf++;
    }

    return STATUS_SUCCESS;
}

status_t LPI2C_MasterReceive(LPI2C_Type *base, uint8_t *rxBuff, size_t rxSize)
{
    status_t result;
    uint8_t *buf;

    //assert(rxBuff);

    /* Handle empty read. */
    if (!rxSize)
    {
        return STATUS_SUCCESS;
    }

    /* Wait until there is room in the command fifo. */
    result = LPI2C_MasterWaitForTxReady(base);
    if (result)
    {
        return result;
    }

    /* Issue command to receive data. */
    base->MTDR = LPI2C_MTDR_CMD(0X1U) | LPI2C_MTDR_DATA(rxSize - 1);

    /* Receive data */
    buf = (uint8_t *)rxBuff;
    while (rxSize--)
    {
        /* Read LPI2C receive fifo register. The register includes a flag to indicate whether */
        /* the FIFO is empty, so we can both get the data and check if we need to keep reading */
        /* using a single register read. */
        uint32_t value;
        do
        {
            /* Check for errors. */
            result = LPI2C_MasterCheckAndClearError(base, LPI2C_MasterGetStatusFlags(base));
            if (result)
            {
                return result;
            }

            value = base->MRDR;
        } while (value & LPI2C_MRDR_RXEMPTY_MASK);

        *buf++ = value & LPI2C_MRDR_DATA_MASK;
    }

    return STATUS_SUCCESS;
}

status_t LPI2C_MasterReceivePN532(LPI2C_Type *base, uint8_t *rxBuff, size_t rxSize)
{
    status_t result;
    uint8_t *buf;

    //assert(rxBuff);

    /* Handle empty read. */
    if (!rxSize)
    {
        return STATUS_SUCCESS;
    }

    /* Wait until there is room in the command fifo. */
    result = LPI2C_MasterWaitForTxReady(base);
    if (result)
    {
        return result;
    }

    /* Issue command to receive data. */
    base->MTDR = LPI2C_MTDR_CMD(0X1U) | LPI2C_MTDR_DATA(rxSize - 1);

    /* Receive data */
    buf = (uint8_t *)rxBuff;
    while (rxSize--)
    {
        /* Read LPI2C receive fifo register. The register includes a flag to indicate whether */
        /* the FIFO is empty, so we can both get the data and check if we need to keep reading */
        /* using a single register read. */
        uint32_t value;
        do
        {
            /* Check for errors. */
            result = LPI2C_MasterCheckAndClearError(base, LPI2C_MasterGetStatusFlags(base));
            if (result)
            {
                return result;
            }

            value = base->MRDR;
        } while (value & LPI2C_MRDR_RXEMPTY_MASK);

        *buf++ = value & LPI2C_MRDR_DATA_MASK;

    }

    return STATUS_SUCCESS;
}

status_t LPI2C_MasterStop(LPI2C_Type *base)
{
    /* Wait until there is room in the fifo. */
    status_t result = LPI2C_MasterWaitForTxReady(base);
    if (result)
    {
        return result;
    }

    /* Send the STOP signal */
    base->MTDR = LPI2C_MTDR_CMD(0x2U);

    /* Wait for the stop detected flag to set, indicating the transfer has completed on the bus. */
    /* Also check for errors while waiting. */
    while (result == STATUS_SUCCESS)
    {
        uint32_t status = LPI2C_MasterGetStatusFlags(base);

        /* Check for error flags. */
        result = LPI2C_MasterCheckAndClearError(base, status);

        /* Check if the stop was sent successfully. */
        if (status & LPI2C_MSR_SDF_MASK)
        {
            LPI2C_MasterClearStatusFlags(base, LPI2C_MSR_SDF_MASK);
            break;
        }
    }

    return result;
}

void LPI2C_MasterSetWatermarks(LPI2C_Type *base, size_t txWords, size_t rxWords)
{
    base->MFCR = LPI2C_MFCR_TXWATER(txWords) | LPI2C_MFCR_RXWATER(rxWords);
}
