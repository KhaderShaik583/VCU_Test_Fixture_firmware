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
#include "drv_mem.h"


#define MPU_NUM_OF_ACC_MASTERS (4U)


/*
    Access right nomenclature:
    access_right_XYZ, where, X - access rights for master0 [CORE], Y - access right for master1[DEBUGGER]
                             Z - access right for master2 [DMA]
    Where X, Y, Z can be - Read-Only (r), Read-Write (rw), Execute (x), Write-Only (w), No-access (n)
    Example, for a region with read-execute only permissions for master0, read-write for debugger and 
    read only for dma - access_right_rxrwro   

*/
static const mpu_master_access_right_t access_right_nrwn[MPU_NUM_OF_ACC_MASTERS] =
{
    /* CORE_0 */
    {
        .masterNum = FEATURE_MPU_MASTER_CORE,
        .accessRight = MPU_SUPERVISOR_USER_NONE,
        .processIdentifierEnable = false
    },
    
    /* DEBUGGER_0 */
    {
        .masterNum = FEATURE_MPU_MASTER_DEBUGGER,
        .accessRight = MPU_SUPERVISOR_USER_RW,
        .processIdentifierEnable = false
    },
    
    /* DMA_0 */
    {
        .masterNum = FEATURE_MPU_MASTER_DMA,
        .accessRight = MPU_SUPERVISOR_USER_NONE,
        .processIdentifierEnable = false
    },
 
    /* ENET_3 */
    {
        .masterNum = FEATURE_MPU_MASTER_ENET,
        .accessRight = MPU_SUPERVISOR_USER_NONE,
        .processIdentifierEnable = false
    }
};

static const mpu_master_access_right_t access_right_rwxrwxrwx[MPU_NUM_OF_ACC_MASTERS] =
{
    /* CORE_1 */
    {
        .masterNum = FEATURE_MPU_MASTER_CORE,
        .accessRight = MPU_SUPERVISOR_USER_RWX,
        .processIdentifierEnable = false
    },
    
    /* DEBUGGER_1 */
    {
        .masterNum = FEATURE_MPU_MASTER_DEBUGGER,
        .accessRight = MPU_SUPERVISOR_USER_RWX,
        .processIdentifierEnable = false
    },
    
    /* DMA_1 */
    {
        .masterNum = FEATURE_MPU_MASTER_DMA,
        .accessRight = MPU_SUPERVISOR_USER_RWX,
        .processIdentifierEnable = false
    },
 
    /* ENET_3 */
    {
        .masterNum = FEATURE_MPU_MASTER_ENET,
        .accessRight = MPU_SUPERVISOR_USER_NONE,
        .processIdentifierEnable = false
    }
    
};
/*! Master access rights configuration 2 */
static const mpu_master_access_right_t access_right_rwrrw[MPU_NUM_OF_ACC_MASTERS] =
{
    /* CORE_2 */
    {
        .masterNum = FEATURE_MPU_MASTER_CORE,
        .accessRight = MPU_SUPERVISOR_USER_RW,
        .processIdentifierEnable = false
    },
    
    /* DEBUGGER_2 */
    {
        .masterNum = FEATURE_MPU_MASTER_DEBUGGER,
        .accessRight = MPU_SUPERVISOR_USER_R,
        .processIdentifierEnable = false
    },
    
    /* DMA_2 */
    {
        .masterNum = FEATURE_MPU_MASTER_DMA,
        .accessRight = MPU_SUPERVISOR_USER_RW,
        .processIdentifierEnable = false
    },
    
    /* ENET_3 */
    {
        .masterNum = FEATURE_MPU_MASTER_ENET,
        .accessRight = MPU_SUPERVISOR_USER_NONE,
        .processIdentifierEnable = false
    }
};
/*! Master access rights configuration 3 */
static const mpu_master_access_right_t access_right_rwrwrw[MPU_NUM_OF_ACC_MASTERS] =
{
    /* CORE_3 */
    {
        .masterNum = FEATURE_MPU_MASTER_CORE,
        .accessRight = MPU_SUPERVISOR_USER_RW,
        .processIdentifierEnable = false
    },
    
    /* DEBUGGER_3 */
    {
        .masterNum = FEATURE_MPU_MASTER_DEBUGGER,
        .accessRight = MPU_SUPERVISOR_USER_RW,
        .processIdentifierEnable = false
    },
    
    /* DMA_3 */
    {
        .masterNum = FEATURE_MPU_MASTER_DMA,
        .accessRight = MPU_SUPERVISOR_USER_RW,
        .processIdentifierEnable = false
    },
    
    /* ENET_3 */
    {
        .masterNum = FEATURE_MPU_MASTER_ENET,
        .accessRight = MPU_SUPERVISOR_USER_RW,
        .processIdentifierEnable = false
    }
};
/*! Master access rights configuration 4 */
static const mpu_master_access_right_t access_right_rxrxrx[MPU_NUM_OF_ACC_MASTERS] =
{
    /* CORE_3 */
    {
        .masterNum = FEATURE_MPU_MASTER_CORE,
        .accessRight = MPU_SUPERVISOR_USER_RX,
        .processIdentifierEnable = false
    },
    
    /* DEBUGGER_3 */
    {
        .masterNum = FEATURE_MPU_MASTER_DEBUGGER,
        .accessRight = MPU_SUPERVISOR_USER_RX,
        .processIdentifierEnable = false
    },
    
    /* DMA_3 */
    {
        .masterNum = FEATURE_MPU_MASTER_DMA,
        .accessRight = MPU_SUPERVISOR_USER_RX,
        .processIdentifierEnable = false
    },

    /* ENET_3 */
    {
        .masterNum = FEATURE_MPU_MASTER_ENET,
        .accessRight = MPU_SUPERVISOR_USER_NONE,
        .processIdentifierEnable = false
    }
};

#define MPU_NUM_OF_REGION_CFG0 (8U)
static const mpu_user_config_t mpu_mem_regions[MPU_NUM_OF_REGION_CFG0] = 
{
    /* Any modification to the scatter file must be cross-checked/updated in below table */
    /* startAddr, endAddr, masterAccRight, processIdentifier, processIdMask */
    
    /* 
        Region0 - Full memory space blocked for core & dma accesses. Regions will be carved into this space in further entries
        Region1 - Vector table with full R/W access in all modes to all masters.
        Region2 - Code RAM for fast functions and flash operations. R/X access to all masters.
        Region3 - SRAM-L region with only R/W access in all modes to all masters. Any code execution will trigger fault.
        Region4 - SRAM-L region with only R/W access in all modes to all masters. Any code execution will trigger fault.
        Region5 - Code Flash with R/W/X access to all masters.
        Region6 - FlexRAM & CSEc PRAM with R/W access only to the core and dma masters. Read access only to debugger.
        Region7 - Heap space with R/W access to all masters. Any code execution will trigger fault.
        Region8 - Stack space with R/W access to all masters. Any code execution will trigger fault.
    */
    
    { 0x00000000U, 0xFFFFFFFFU, access_right_nrwn,      0x00U, 0x00U },      /* Region 0 full memory space */
    { 0x1FFE0000U, 0x1FFF03ffU, access_right_rwxrwxrwx, 0x00U, 0x00U },      /* Region 1 Vector Table - If vector table is in RAM ONLY. Else comment this region.  */
    { 0x1FFE0400U, 0x1FFF43FFU, access_right_rxrxrx,    0x00U, 0x00U },      /* Region 2 Code RAM for fast functions and flash operations */
    { 0x1FFF4400U, 0x1fffffffU, access_right_rwrwrw,    0x00U, 0x00U },      /* Region 3 SRAM-L */
    { 0x20000000U, 0x2000ffffU, access_right_rwrwrw,    0x00U, 0x00U },      /* Region 4 SRAM-U [Must be adjusted if stack size changes] */
    
    { 0x00040400U, 0x001207ffU, access_right_rwxrwxrwx, 0x00U, 0x00U },      /* Region 5 Code flash + OTA */  
    { 0x14000000U, 0x1400107fU, access_right_rwrrw,     0x00U, 0x00U },      /* Region 6 FlexRAM & CSEc PRAM */
    
    { 0x20010000U, 0x200127ffU, access_right_rwrwrw,    0x00U, 0x00U },      /* Region 7 Heap */

};

void mpu_config(void)
{
    status_t rc = STATUS_ERROR;

    __DMB();
    
    rc = MPU_DRV_Init(0U, MPU_NUM_OF_REGION_CFG0, mpu_mem_regions);
    
    __DSB();
    __ISB();
    
    DEV_ASSERT(rc == STATUS_SUCCESS);
}

void mpu_protect_vector(void)
{
    status_t rc = STATUS_SUCCESS;
    
   __DMB();
    
    rc = MPU_DRV_SetMasterAccessRights(0U, 1U, access_right_rxrxrx);
    DEV_ASSERT(rc == STATUS_SUCCESS);
    
    __DSB();
    __ISB();
    
    MPU_DRV_EnableRegion(0U, 1U, true);
}

void mpu_unprotect_vector(void)
{
    status_t rc = STATUS_SUCCESS;
    
    __DMB();
    
    rc = MPU_DRV_SetMasterAccessRights(0U, 1U, access_right_rwxrwxrwx);
    DEV_ASSERT(rc == STATUS_SUCCESS);
    
    __DSB();
    __ISB();
    
    MPU_DRV_EnableRegion(0U, 1U, true);
}
