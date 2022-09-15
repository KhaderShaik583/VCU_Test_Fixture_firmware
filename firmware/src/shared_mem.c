 /*
 * 
 * ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
 * ______________________________________
 * 
 * [2019] - [2020] Ultraviolette Automotive Pvt. Ltd.
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
 
#include "shared_mem.h"
#include "eth_task.h"

#define SHMEM_NUM_MEMPOOL_OBJECTS           (10U)
#define SHMEM_NUM_SWIF_MEMPOOL_OBJECTS      (10U)

static osMemoryPoolId_t shmem_pool_ids[SHMEM_MAX_POOL_TYPES];

osMemoryPoolId_t shmem_create_pool(shmem_pool_types_e type)
{
    osMemoryPoolId_t mem_pool_id = NULL;
    
    if(type == SHMEM_POOL_TYPE_BMS)
    {
        mem_pool_id = osMemoryPoolNew(SHMEM_NUM_MEMPOOL_OBJECTS, sizeof(shmem_block_bms_t), NULL);
        DEV_ASSERT(mem_pool_id != NULL);
        
        shmem_pool_ids[type] = mem_pool_id;
    }
    else if(type == SHMEM_POOL_TYPE_SWIF)
    {
        mem_pool_id = osMemoryPoolNew(SHMEM_NUM_SWIF_MEMPOOL_OBJECTS, sizeof(shmem_block_swif_t), NULL);
        DEV_ASSERT(mem_pool_id != NULL);
        
        shmem_pool_ids[type] = mem_pool_id; 
    }
    else 
    {
        mem_pool_id = NULL;
        __NOP();
    }
    
    return mem_pool_id;
}

void *shmem_alloc_block(shmem_pool_types_e type)
{
    void *blk = NULL;
    
    if(type == SHMEM_POOL_TYPE_BMS)
    {
        blk = osMemoryPoolAlloc(shmem_pool_ids[type], 0U);
        /* DEV_ASSERT(blk != NULL); */
    }
    else if(type == SHMEM_POOL_TYPE_SWIF)
    {
        blk = osMemoryPoolAlloc(shmem_pool_ids[type], 0U);
        /* DEV_ASSERT(blk != NULL);*/
        /* For switch interface ignore */
    }
    else
    {
        __NOP();
    }
    
    return blk;
}

void shmem_free_block(shmem_pool_types_e type, void *blk)
{
    osStatus_t status;
    
    if(type == SHMEM_POOL_TYPE_BMS)
    {
        status = osMemoryPoolFree(shmem_pool_ids[type], blk);
        DEV_ASSERT(status == osOK);
    }
    else if(type == SHMEM_POOL_TYPE_SWIF)
    {
        status = osMemoryPoolFree(shmem_pool_ids[type], blk);
        DEV_ASSERT(status == osOK);
    }
    else
    {
        __NOP();
    }
        
}

uint32_t shmem_get_capacity(shmem_pool_types_e type)
{
    uint32_t available_blocks = 0U;
    
    available_blocks = osMemoryPoolGetSpace(shmem_pool_ids[type]);
    
    return available_blocks;
}
