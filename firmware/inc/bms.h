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
 
#ifndef BMS_H
#define BMS_H

#include "fw_common.h"
#include "batterypack.h"
#include "bms_stat.h"
#include "rtc_task.h"
#include "udp_task.h"
#include "shared_mem.h"
#include "init_task.h"
#include "dba_task.h"
#include "odometer_task.h" 
#include "drv_loadswitches.h"
#include "bcm.h"
#include "mc_task.h"
#include "shared_mem.h"

#define CAN_MSG_TYPE_SET    1U
#define CAN_MSG_TYPE_GET    2U
#define CAN_MSG_TYPE_NTF    3U


/* Definition of the TX and RX message buffers depending on the bus role */
#define CANFD_STATE_INIT_RX         (0U)
#define CANFD_STATE_WAIT_RX_NB      (1U)

#define CAN_BMS_TX_MAILBOX          (1U)
#define CAN_BMS_TX_MSG_ID           (0x200U)
#define CAN_BMS_RX_MAILBOX          (2U)
#define CAN_BMS_RX_MSG_ID           (0x8000U)

#define CAN_FD_MAX_LEN              (64U)
#define CAN_MSG_PRIO_SHIFT          (26U)
#define CAN_MSG_SLOT_ID_SHIFT       (18U)
#define CAN_MSG_MSG_ID_SHIFT        (6U)

#define BMS_FET_CTRL_MSG_ID     (3U)
#define BMS_FET_ON_MSG          (0xAFU)

#define MAX_CANFD_LOGICAL_INTERFACES  (1U)

#define CANFD_BUS_TRANSACTION_PENDING   (0U)
#define CANFD_BUS_TRANSACTION_DONE      (1U)

#define CANFD_LOGICAL_BUS0  (0U)
#define CANFD_LOGICAL_BUS1  (1U)
#define CANFD_LOGICAL_BUS2  (2U)

#define CANFD_LOGICAL_BUS_ENABLED      (1U)
#define CANFD_LOGICAL_BUS_DISABLED     (0U)

#define BMS_VCU_SLOT0_COMM_ID           (0x200000U)
#define BMS_VCU_MBX_GBL_MASK_FILTER     (0xE00000U)
#define CAN_FD_BUS_TIMEOUT              (500U)

#define CAN_IF_MSG_GET_CELLS_ID                 (0x904U)
#define CAN_IF_MSG_GET_TEMPERATURE_INFO_ID      (0x901U)
#define CAN_IF_MSG_GET_CM_DIAG_INFO_ID          (0x902U)
#define CAN_IF_MSG_GET_FG_INFO_ID               (0x903U)   
#define CAN_IF_MSG_GET_EM_INFO_ID               (0x900U)
#define CAN_IF_MSG_GET_SENSOR_INFO_ID           (0x905U)
#define CAN_IF_MSG_BMS_STATUS_ID                (0x906U)
#define CAN_IF_MSG_BAL_INFO_ID                  (0x907U)
#define CAN_IF_MSG_GET_FW_INFO_ID               (0x908U)
#define CAN_IF_MSG_GET_LAST_RST_STATE_ID        (0x909U)
#define CAN_IF_MSG_GET_UID_ID                   (0x910U)
#define CAN_IF_MSG_NTF_INIT_COMPLETE_ID         (0x911U)
#define CAN_IF_MSG_GET_EXCP_ID                  (0x912U)
#define CAN_IF_MSG_GET_SLOT_VTG_ID              (0x913U)
#define CAN_IF_MSG_GET_LOG_FS_SZ_ID             (0x914U)
#define CAN_IF_MSG_NTF_BMS_RDY_ID               (0x915U)
#define CAN_IF_MSG_CM_PART2_ID                  (0x916U)
#define CAN_IF_MSG_BMS_HEART_BEAT_ID            (0x917U)
#define CAN_IF_MSG_BMS_READ_TIME_ID             (0x918U)
#define CAN_IF_MSG_BMS_WAKE_NTF_ID              (0x919U)
#define CAN_IF_MSG_DSG_CHG_FET_ON_ID            (0x920U)
#define CAN_IF_MSG_DSG_CHG_FET_OFF_ID           (0x921U)
#define CAN_IF_MSG_DSG_FET_ON_ID                (0x922U)
#define CAN_IF_MSG_DSG_FET_OFF_ID               (0x923U)
#define CAN_IF_MSG_CHG_FET_ON_ID                (0x924U)
#define CAN_IF_MSG_CHG_FET_OFF_ID               (0x925U)
#define CAN_IF_MSG_ERR_RESPONSE_ID              (0x926U)
#define CAN_IF_MSG_ID_WAKEUP_BMS_ID             (0x927U)
#define CAN_IF_MSG_BMS_EXECEPTION_INFO_ID       (0x928U)
#define CAN_IF_MSG_BMS_DSG_OC_INFO_ID           (0x929U)
#define CAN_IF_MSG_BMS_CHG_OC_INFO_ID           (0x92AU)
#define CAN_IF_MSG_BMS_DSG_UV_INFO_ID           (0x92BU)
#define CAN_IF_MSG_BMS_CHG_OV_INFO_ID           (0x92CU)
#define CAN_IF_MSG_BMS_PEAK_DSG_I_INFO_ID       (0x92DU)
#define CAN_IF_MSG_BMS_BF_BAL_CELL_INFO_ID      (0x92EU)
#define CAN_IF_MSG_BMS_AF_BAL_CELL_INFO_ID      (0x92FU)
#define CAN_IF_MSG_BMS_FORCE_BAL_SHTDN_ID       (0x930U)
#define CAN_IF_MSG_BMS_PROC_EXEC_ERR_ID         (0x932U)
#define CAN_IF_MSG_BMS_FW_UPD_MSG_ID            (0x933U)
#define CAN_IF_MSG_BMS_FW_UPD_DONE_MSG_ID       (0x934U)
#define CAN_IF_MSG_BATTERY_PACK_INFO_MSG_ID     (0x935U)
#define CAN_IF_MSG_EXCP_NTF_MSG_ID              (0x936U)
#define CAN_IF_MSG_BMS_RST_FG_ID                (0x938U)
#define CAN_IF_MSG_BMS_SET_PWR_MODE_ID          (0x939U)

#define CAN_IF_MSG_BMS_GET_EC_AT_BOOT_ID        (0x941U)
#define CAN_IF_MSG_RST_TAMPER_SENTINEL_ID       (0x942U)

typedef enum
{
    CAN_IF_MSG_ID_GET_EM_INFO = 0U,
    CAN_IF_MSG_ID_GET_TEMPERATURE_INFO,
    CAN_IF_MSG_ID_GET_CM_DIAG_INFO,
    CAN_IF_MSG_ID_GET_FG_INFO,
    CAN_IF_MSG_ID_GET_CELLS_INFO,
    CAN_IF_MSG_ID_GET_SENSOR_INFO,
    CAN_IF_MSG_ID_BMS_STATUS,
    CAN_IF_MSG_ID_BAL_INFO,
    CAN_IF_MSG_GET_FW_INFO,
    CAN_IF_MSG_GET_LAST_RST_STATE,
    CAN_IF_MSG_GET_UID,
    CAN_IF_MSG_NTF_INIT_COMPLETE,
    CAN_IF_MSG_GET_EXCP,
    CAN_IF_MSG_GET_SLOT_VTG,
    CAN_IF_MSG_GET_LOG_FS_SZ,
    CAN_IF_MSG_NTF_BMS_RDY,
    CAN_IF_MSG_CM_PART2,
    CAN_IF_MSG_BMS_HEART_BEAT,
    CAN_IF_MSG_BMS_READ_TIME,
    CAN_IF_MSG_BMS_WAKE_NTF,
    CAN_IF_MSG_DSG_CHG_FET_ON,      
    CAN_IF_MSG_DSG_CHG_FET_OFF,     
    CAN_IF_MSG_DSG_FET_ON,           
    CAN_IF_MSG_DSG_FET_OFF,         
    CAN_IF_MSG_CHG_FET_ON,         
    CAN_IF_MSG_CHG_FET_OFF,    
    CAN_IF_MSG_BMS_EXECEPTION_INFO,
    CAN_IF_MSG_BMS_DSG_OC_INFO,
    CAN_IF_MSG_BMS_CHG_OC_INFO,
    CAN_IF_MSG_BMS_DSG_UV_INFO,
    CAN_IF_MSG_BMS_CHG_OV_INFO,   
    CAN_IF_MSG_BMS_PEAK_DSG_I_INFO,
    CAN_IF_MSG_BMS_BF_BAL_CELL_INFO,
    CAN_IF_MSG_BMS_AF_BAL_CELL_INFO,
    CAN_IF_MSG_BMS_FORCE_BAL_SHTDN,
    CAN_IF_MSG_BMS_FW_UPD_MSG,
    CAN_IF_MSG_BMS_FW_UPD_DONE_MSG,
    CAN_IF_MSG_GET_BATT_INFO,
    CAN_IF_MSG_HW_RST_FG,
    CAN_IF_MSG_BMS_SET_PWR_MODE,
    CAN_IF_MSG_BMS_GET_EC_AT_BOOT,
    CAN_IF_MSG_RST_TAMPER_SENTINEL,
    
    /* Do not add message ids beyond this line */
    CAN_IF_MAX_MSG_IDS
}can_messages_e;

#endif /* BMS_H */
