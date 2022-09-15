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

#ifndef NVM_H
#define NVM_H

#include "fw_common.h"
#include "osif.h"
#include "rtx_os.h"
#include "rtc_task.h"

#define NVM_FILE_SECTOR_BYTES			(256U)
#define NVM_FLEX_PAGE_SIZE				(4096U)

#define NVM_MAX_TRIPS                   (5U)
#define NVM_WAIT_TIMEOUT                (4096U)

/*
		The NVM is configured in the FlexNVM section
*/
#define	FILE_SECT_BOOT_CONFIGURATION    (uint32_t)(0U)
#define	FILE_SECT_USER_IDENTIFICATION   (uint32_t)(FILE_SECT_BOOT_CONFIGURATION  + NVM_FILE_SECTOR_BYTES)
#define FILE_SECT_ODO_PERSISTENCE       (uint32_t)(FILE_SECT_USER_IDENTIFICATION + NVM_FILE_SECTOR_BYTES)
#define FILE_SECT_RTC_TIME_SENTINEL     (uint32_t)(FILE_SECT_ODO_PERSISTENCE + NVM_FILE_SECTOR_BYTES)
#define FILE_SECT_COREDUMP_INFO         (FILE_SECT_RTC_TIME_SENTINEL + NVM_FILE_SECTOR_BYTES)
#define FILE_SECT_IMU_CAL_INFO          (FILE_SECT_COREDUMP_INFO + NVM_FILE_SECTOR_BYTES)
#define FILE_SECT_MC_TORQUE_LR_MAPS     (FILE_SECT_IMU_CAL_INFO + NVM_FILE_SECTOR_BYTES)
#define FILE_SECT_MC_TORQUE_HR_MAPS     (FILE_SECT_MC_TORQUE_LR_MAPS + NVM_FILE_SECTOR_BYTES)
#define FILE_SECT_MC_TORQUE_PA_MAPS     (FILE_SECT_MC_TORQUE_HR_MAPS + NVM_FILE_SECTOR_BYTES)
#define FILE_SECT_MC_CTXT               (FILE_SECT_MC_TORQUE_PA_MAPS + NVM_FILE_SECTOR_BYTES)
#define FILE_SECT_OS_ERR_CTXT           (FILE_SECT_MC_CTXT + NVM_FILE_SECTOR_BYTES)

#define FILE_SECT_BOOT_FW_UPD_OFFSET            (16U)

typedef struct
{
    char username[32];
    char licencenum[16];
    uint32_t uiid;
    uint32_t giid;
}nv_user_t;

typedef struct 
{
    float_t distance;
    float_t watt_hour;
    float_t trip_duration;
    float_t average_speed;
}trip_info_t;

typedef struct {
    uint32_t sentinel;
    float_t odometer;
    trip_info_t trip[NVM_MAX_TRIPS];
}odometer_persistence_t;

status_t nvm_init(void);
void nvm_user_config(uint8_t *nvbuffer);
void nvm_get_user_config(uint8_t *nvbuffer);
status_t nvm_write(uint32_t sector, const uint8_t *nvm_buffer, uint32_t num_bytes);
status_t nvm_write_byte(uint32_t sector, uint8_t nvm_buffer);
void nvm_read(uint32_t sector, uint8_t *nvm_buffer, uint32_t num_bytes);
uint32_t *nvm_get_file_base_addr(uint32_t sector);
void nvm_dumpinfo(const char *file, const char *func, uint32_t lineno);
void nvm_write_err(uint32_t sector, const uint8_t *nvm_buffer, uint32_t num_bytes);
#endif /* NVM_H */

