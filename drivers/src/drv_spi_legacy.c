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
 * Author : Rishi F. [011]
 *


 */
 
#include "fw_common.h"
#include "board.h"
#include "lpspi_master_driver.h"
#include "lpspi_hw_access.h"
#include "pins_driver.h"
#include "drv_spi_legacy.h"

uint8_t lpspi_trancieve_sja_read32(uint8_t deviceSelect, uint8_t wordCount, uint32_t registerAddress, uint32_t *p_registerValue)
{
    uint8_t ret = STATUS_SUCCESS;
    uint32_t i = 0U;
    uint32_t wordToSend = 0U;
    uint32_t dummy = 0U;
    
    UNUSED_PARAM(deviceSelect);
    
    /* Assert  CS */
    PINS_DRV_ClearPins(ENET_CS_GPIO, (1U << ENET_CS_PIN));
    
    /* Check that we're not busy. */
    if (LPSPI_GetStatusFlag(LPSPI2, LPSPI_MODULE_BUSY))
    {
        ret = STATUS_BUSY;
    }
    #ifdef ERRATA_E10655
    else
    {
        /* Double check to fix errata e10655. */
        if (LPSPI_GetStatusFlag(LPSPI1, LPSPI_MODULE_BUSY))
        {
            ret = STATUS_BUSY;
        }
    }
    #endif
    
    if(ret == STATUS_SUCCESS)
    {
        /* Configure watermarks */
        LPSPI_SetRxWatermarks(LPSPI2, 0U);
        LPSPI_SetTxWatermarks(LPSPI2, 1U);
        
        (void)LPSPI_ClearStatusFlag(LPSPI2, LPSPI_ALL_STATUS);
        LPSPI_SetFlushFifoCmd(LPSPI2, true, true);
        LPSPI_SetFlushFifoCmd(LPSPI2, true, true);   
        
        i = 0U;
        wordToSend = 0U;
        
        /* Send Address */
        wordToSend = (registerAddress << 4U) | ((uint32_t)wordCount << 25U);
        wordToSend = wordToSend & (0x7fffffff);
        while((LPSPI2->SR & LPSPI_SR_TDF_MASK)>>LPSPI_SR_TDF_SHIFT == 0);  
        LPSPI_WriteData(LPSPI2, wordToSend);
        
        while((LPSPI2->SR & LPSPI_SR_TCF_MASK)>>LPSPI_SR_TCF_SHIFT == 0);
        while((LPSPI2->SR & LPSPI_SR_RDF_MASK)>>LPSPI_SR_RDF_SHIFT == 0);
        
        dummy = LPSPI2->RDR;

        (void)LPSPI_ClearStatusFlag(LPSPI2, LPSPI_ALL_STATUS);
        
        while(wordCount > 0U)
        {
            while((LPSPI2->SR & LPSPI_SR_TDF_MASK)>>LPSPI_SR_TDF_SHIFT == 0);
            
            LPSPI_WriteData(LPSPI2, 0xFFFFFFFF);
                
            while((LPSPI2->SR & LPSPI_SR_TCF_MASK)>>LPSPI_SR_TCF_SHIFT == 0);

            while((LPSPI2->SR & LPSPI_SR_RDF_MASK)>>LPSPI_SR_RDF_SHIFT == 0);

            p_registerValue[i] = LPSPI2->RDR;
            LPSPI2->SR |= LPSPI_SR_RDF_MASK; 

            i = i + 1U;
            wordCount = wordCount - 1U;
        }
        
        (void)LPSPI_ClearStatusFlag(LPSPI2, LPSPI_ALL_STATUS);  

    }
    
    /* De-assert CS */
    PINS_DRV_SetPins(ENET_CS_GPIO, (1U << ENET_CS_PIN));
    
    return ret; 
}

uint8_t lpspi_trancieve_sja_write32(uint8_t deviceSelect, uint8_t wordCount, uint32_t registerAddress, uint32_t *p_registerValue)
{
    uint8_t ret = STATUS_SUCCESS;
    uint32_t i = 0u;
    uint32_t wordToSend = 0U;
    
    UNUSED_PARAM(deviceSelect);
    
    /* Assert  CS */
    PINS_DRV_ClearPins(ENET_CS_GPIO, (1U << ENET_CS_PIN));

    /* Check that we're not busy. */
    if (LPSPI_GetStatusFlag(LPSPI2, LPSPI_MODULE_BUSY))
    {
        ret = STATUS_BUSY;
    }
    #ifdef ERRATA_E10655
    else
    {
        /* Double check to fix errata e10655. */
        if (LPSPI_GetStatusFlag(LPSPI0, LPSPI_MODULE_BUSY))
        {
            ret = STATUS_BUSY;
        }
    }
    #endif
    
    if(ret == STATUS_SUCCESS)
    {
        /* Configure watermarks */
        LPSPI_SetRxWatermarks(LPSPI2, 0U);
        LPSPI_SetTxWatermarks(LPSPI2, 1U);
        
        (void)LPSPI_ClearStatusFlag(LPSPI2, LPSPI_ALL_STATUS);
        LPSPI_SetFlushFifoCmd(LPSPI2, true, true);
        LPSPI_SetFlushFifoCmd(LPSPI2, true, true);
        
        /* Ignore RX data */
        LPSPI2->TCR |= LPSPI_TCR_RXMSK_MASK;
        
        /* Send Address */
        wordToSend = (registerAddress << 4U) | 0x80000000U;
        while((LPSPI2->SR & LPSPI_SR_TDF_MASK)>>LPSPI_SR_TDF_SHIFT == 0);
            
        LPSPI_WriteData(LPSPI2, wordToSend);
            
        while((LPSPI2->SR & LPSPI_SR_TCF_MASK)>>LPSPI_SR_TCF_SHIFT == 0);
            
        LPSPI2->SR |= LPSPI_SR_TDF_MASK; 
        LPSPI2->SR |= LPSPI_SR_FCF_MASK;
        LPSPI2->SR |= LPSPI_SR_TCF_MASK;
            
        while(wordCount > 0U)
        {
            wordToSend = p_registerValue[i];
            
            while((LPSPI2->SR & LPSPI_SR_TDF_MASK)>>LPSPI_SR_TDF_SHIFT == 0);
            
            LPSPI_WriteData(LPSPI2, wordToSend);
            
            while((LPSPI2->SR & LPSPI_SR_TCF_MASK)>>LPSPI_SR_TCF_SHIFT == 0);
            
            LPSPI2->SR |= LPSPI_SR_TDF_MASK; 
            LPSPI2->SR |= LPSPI_SR_FCF_MASK;
            LPSPI2->SR |= LPSPI_SR_TCF_MASK;

            i = i + 1U;
            wordCount = wordCount - 1U;
            wordToSend = 0U;
        }
        
        (void)LPSPI_ClearStatusFlag(LPSPI2, LPSPI_ALL_STATUS);
        LPSPI2->TCR &= ~LPSPI_TCR_RXMSK_MASK;
    } 
    
    /* De-assert CS */
    PINS_DRV_SetPins(ENET_CS_GPIO, (1U << ENET_CS_PIN));
    
    return ret;
}

uint8_t lpspi_write8_icm20948(uint8_t reg, const uint8_t *tx_data, uint8_t tx_len)
{
    uint8_t ret = STATUS_SUCCESS;
    volatile uint32_t address = 0U;
    volatile uint32_t data = 0U;
    uint32_t dummy = 0U;
    uint32_t i = 0U;

    /* Check that we're not busy. */
    if(LPSPI_GetStatusFlag(LPSPI1, LPSPI_MODULE_BUSY))
    {
        ret = STATUS_BUSY;
    }

    /* Configure watermarks */
    LPSPI_SetRxWatermarks(LPSPI1, 1U);
    LPSPI_SetTxWatermarks(LPSPI1, 1U);
    
    if(ret == STATUS_SUCCESS)
    {
        (void)LPSPI_ClearStatusFlag(LPSPI1, LPSPI_ALL_STATUS);
        
        LPSPI_SetFlushFifoCmd(LPSPI1, true, true);
        LPSPI_SetFlushFifoCmd(LPSPI1, true, true);

        address = reg;
        
        PINS_DRV_ClearPins(IMU_CS_GPIO, 1U << IMU_CS_PIN);
        
        LPSPI_WriteData(LPSPI1, address);
        
        while(tx_len > 0U)
        {
            data = tx_data[i];
            LPSPI_WriteData(LPSPI1, data);
            
            while((LPSPI1->SR & LPSPI_SR_TDF_MASK)>>LPSPI_SR_TDF_SHIFT == 0);
            while((LPSPI1->SR & LPSPI_SR_TCF_MASK)>>LPSPI_SR_TCF_SHIFT == 0);
            while((LPSPI1->SR & LPSPI_SR_RDF_MASK)>>LPSPI_SR_RDF_SHIFT == 0);
            /* NOTE: If IMU messes up, make dummy uint8_t */
            dummy = LPSPI1->RDR;

            LPSPI1->SR |= LPSPI_SR_TDF_MASK; 
            LPSPI1->SR |= LPSPI_SR_FCF_MASK;
            LPSPI1->SR |= LPSPI_SR_RDF_MASK; 

            tx_len = tx_len - 1U;
            i++;

            (void)LPSPI_ClearStatusFlag(LPSPI1, LPSPI_ALL_STATUS);
        }
        
        PINS_DRV_SetPins(IMU_CS_GPIO, 1U << IMU_CS_PIN);
    }     
    
    return ret;
}

uint8_t lpspi_read8_icm20948(uint8_t reg, uint8_t *rx_data, uint8_t rx_len)
{
    uint8_t ret = STATUS_SUCCESS;
    volatile uint8_t wordToSend = 0xFF;
    volatile uint32_t i = 0U;
    
    /* Check that we're not busy. */
    if (LPSPI_GetStatusFlag(LPSPI1, LPSPI_MODULE_BUSY))
    {
        ret = STATUS_BUSY;
    }
        
    /* Configure watermarks */
    LPSPI_SetRxWatermarks(LPSPI1, 2U);
    LPSPI_SetTxWatermarks(LPSPI1, 0U);
    
    if(ret == STATUS_SUCCESS)
    {
        (void)LPSPI_ClearStatusFlag(LPSPI1, LPSPI_ALL_STATUS);
        
        LPSPI_SetFlushFifoCmd(LPSPI1, true, true);
        LPSPI_SetFlushFifoCmd(LPSPI1, true, true);

        /* Send Address */
        wordToSend = reg;
        
        PINS_DRV_ClearPins(IMU_CS_GPIO, 1U << IMU_CS_PIN);
        LPSPI_WriteData(LPSPI1, wordToSend);
        while((LPSPI1->SR & LPSPI_SR_RDF_MASK)>>LPSPI_SR_RDF_SHIFT == 0);
        (void)LPSPI1->RDR;
        
        wordToSend = 0xFF;
        while(i < rx_len)
        {         
            LPSPI_WriteData(LPSPI1, wordToSend);

            while((LPSPI1->SR & LPSPI_SR_TCF_MASK)>>LPSPI_SR_TCF_SHIFT == 0);
            while((LPSPI1->SR & LPSPI_SR_RDF_MASK)>>LPSPI_SR_RDF_SHIFT == 0);
            
            rx_data[i] = (uint8_t)LPSPI1->RDR;

            LPSPI1->SR |= LPSPI_SR_RDF_MASK; 
            LPSPI1->SR |= LPSPI_SR_TDF_MASK; 
            LPSPI1->SR |= LPSPI_SR_FCF_MASK;
            i++;
            wordToSend = 0xFF;            
        }
   
        LPSPI1->SR |= LPSPI_SR_TDF_MASK; 
        LPSPI1->SR |= LPSPI_SR_FCF_MASK;
        (void)LPSPI_ClearStatusFlag(LPSPI1, LPSPI_ALL_STATUS);
        
        PINS_DRV_SetPins(IMU_CS_GPIO, 1U << IMU_CS_PIN);
    }     
    
    return ret;    
}

