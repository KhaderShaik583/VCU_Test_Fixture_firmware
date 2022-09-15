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
 
#ifndef SHARED_MEM_H
#define SHARED_MEM_H

#include "fw_common.h"
#include "rtx_os.h"
#include "udp_task.h"
#include "imu.h"

#define SHMEM_BMS_BLK_SIZE  (512U)

typedef enum
{
    SHMEM_POOL_TYPE_BMS = 0U,
    SHMEM_POOL_TYPE_SWIF,
    SHMEM_POOL_TYPE_ABS,
    SHMEM_MAX_POOL_TYPES
}shmem_pool_types_e;

typedef struct
{
    udp_msg_t udp_msg;
    uint8_t buffer[SHMEM_BMS_BLK_SIZE];
}shmem_block_bms_t;

typedef struct
{
    udp_msg_t udp_msg;
    uint8_t sw_info;
}shmem_block_swif_t;


typedef struct
{
    udp_msg_t udp_msg;
    imu_data_t imu_measurement;
}shmem_block_imu_t;

osMemoryPoolId_t shmem_create_pool(shmem_pool_types_e type);
void *shmem_alloc_block(shmem_pool_types_e type);
void shmem_free_block(shmem_pool_types_e type, void *blk);
uint32_t shmem_get_capacity(shmem_pool_types_e type);
#endif /* SHARED_MEM_H */
