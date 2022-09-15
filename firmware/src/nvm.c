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
#include "nvm.h"

static const volatile uint32_t flexram_base = 0x14000000U;

static semaphore_t nvm_sem_t;

static void nvm_wait(void)
{
    volatile uint8_t fstat = 0U;
    volatile uint32_t timeout = 0U;
    
	fstat = FTFC->FSTAT;
    while((fstat & FTFC_FSTAT_CCIF_MASK) != FTFC_FSTAT_CCIF_MASK)
	{
        timeout++;
        if(timeout > NVM_WAIT_TIMEOUT)
        {
            break;
        }
		
		fstat = FTFC->FSTAT;
	}
}

static status_t nvm_sem_acquire(uint32_t timeout)
{
    status_t s;
    s = osif_sem_acquire(&nvm_sem_t, timeout);
    return s;
}

static status_t nvm_sem_release(void)
{
    status_t s;
    s = osif_sem_release(&nvm_sem_t);
    return s;
}

static void nvm_sem_init(void)
{
    status_t status = STATUS_ERROR;
    
    status = osif_sem_create(&nvm_sem_t, 0U);
    DEV_ASSERT(status == STATUS_SUCCESS);
}

status_t nvm_init(void)
{
    status_t result = STATUS_SUCCESS;
    volatile uint8_t fcnfg = 0U;
    
    fcnfg = FTFC->FCNFG;
    
    /* Check if flash is partitioned */
    if((fcnfg & FTFC_FCNFG_EEERDY_MASK) == FTFC_FCNFG_EEERDY_MASK)
    {
        /* Check the NVM Secure Flag Setting */
        nvm_wait();
        
        if(*(volatile uint8_t *)flexram_base == 0xA5)
        {
            /* Flash is partitioned & CSEC keys are configured */
            result = STATUS_SUCCESS;
        }
        else
        {
            result = STATUS_ERROR;  
        }
        
        nvm_wait();
    }
    
    nvm_sem_init();
    
    return result;
}

status_t nvm_write(uint32_t sector, const uint8_t *nvm_buffer, uint32_t num_bytes)
{
    status_t result = STATUS_SUCCESS;
    int32_t lc = 0;
    
	uint8_t *file_address = NULL;
	uint32_t index = 0U;
    
    DEV_ASSERT(nvm_buffer != NULL);
    DEV_ASSERT((num_bytes - 1U) < NVM_FILE_SECTOR_BYTES);

    lc = osif_enter_critical();
	
	file_address = (uint8_t *)(flexram_base + sector);
    nvm_wait();
        
    for(index = 0; index < num_bytes; index++)
	{
        nvm_wait();
        
		*file_address = nvm_buffer[index];
        file_address++;
	}
    
    nvm_wait();
        
	(void)osif_exit_critical(lc);
		
	return result;
}

status_t nvm_write_byte(uint32_t sector, uint8_t nvm_buffer)
{
    status_t result = STATUS_SUCCESS;
	uint8_t *file_address = NULL;
    int32_t lc = 0;
    
    lc = osif_enter_critical();
    
    file_address = (uint8_t *)(flexram_base + sector);
    nvm_wait();

    *file_address = nvm_buffer;
    nvm_wait();

	(void)osif_exit_critical(lc);
		
	return result;
}

void nvm_read(uint32_t sector, uint8_t *nvm_buffer, uint32_t num_bytes)
{
	uint32_t index = 0U;
    int32_t lc = 0;
    
    DEV_ASSERT(nvm_buffer != NULL);
    DEV_ASSERT((num_bytes - 1U) < NVM_FILE_SECTOR_BYTES);
	DEV_ASSERT(sector < NVM_FLEX_PAGE_SIZE);

    lc = osif_enter_critical();
    
    nvm_wait();

    for(index = 0U; index < num_bytes; index++)
    {
       nvm_wait();

       *(nvm_buffer + index) = *(uint8_t *)((flexram_base + sector) + index);
    }
    
    nvm_wait();

	(void)osif_exit_critical(lc);
}

uint32_t *nvm_get_file_base_addr(uint32_t sector)
{
    /* TODO: Check for sector validity */
    
    return ((uint32_t *)(sector + flexram_base));
}

void nvm_write_err(uint32_t sector, const uint8_t *nvm_buffer, uint32_t num_bytes)
{    
	uint8_t *file_address = NULL;
	uint32_t index = 0U;
    
    DEV_ASSERT(nvm_buffer != NULL);
    DEV_ASSERT((num_bytes - 1U) < NVM_FILE_SECTOR_BYTES);
    
    INT_SYS_DisableIRQGlobal();
    
	file_address = (uint8_t *)(flexram_base + sector);
    nvm_wait();
        
    for(index = 0; index < num_bytes; index++)
	{
        nvm_wait();
        
		*file_address = nvm_buffer[index];
        file_address++;
	}
    
    nvm_wait();
    
    INT_SYS_EnableIRQGlobal();

}
