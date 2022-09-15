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
 
#include "can_messenger_tx.h"
#include "can_messenger_rx.h"
#include "batterypack.h"
#include "stat.h"
#include "bms_can_if.h"
#include "bms_task.h"
#include "init_msg.h"
#include "fota.h" 

#define CANFD_MSG_SIG_LEN   (8U) 

/* For compatibility with earlier designs that had 3 ports for 3 packs */
static const uint32_t slot_to_msg_id_map[MAX_CANFD_LOGICAL_INTERFACES] = {
        /* PORT 0 */ 0x200000U
};

static uint8_t fw_msg_sig[CANFD_MSG_SIG_LEN + 56U];
static volatile uint32_t mscm_ocmdr0_save = 0U;
static volatile uint32_t mscm_ocmdr1_save = 0U;

static status_t canfd_send_CAN_IF_MSG_GET_EM_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_ID_GET_TEMPERATURE_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_ID_GET_CM_DIAG_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_ID_GET_FG_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_ID_GET_CELLS_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_ID_GET_SENSOR_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_ID_BMS_STATUS(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_ID_BAL_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_GET_FW_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_GET_LAST_RST_STATE(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_GET_UID(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_NTF_INIT_COMPLETE(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_GET_EXCP(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_GET_SLOT_VTG(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_GET_LOG_FS_SZ(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_NTF_BMS_RDY(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_CM_PART2(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BMS_HEART_BEAT(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BMS_READ_TIME(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BMS_WAKE_NTF(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_DSG_CHG_FET_ON(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_DSG_CHG_FET_OFF(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_DSG_FET_ON(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_DSG_FET_OFF(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_CHG_FET_ON(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_CHG_FET_OFF(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BMS_EXECEPTION_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BMS_DSG_OC_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BMS_CHG_OC_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BMS_DSG_UV_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BMS_CHG_OV_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BMS_PEAK_DSG_I_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BMS_BF_BAL_CELL_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BMS_AF_BAL_CELL_INFO(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BMS_FORCE_BAL_SHTDN(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BMS_FW_UPD_MSG(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BMS_FW_UPD_DONE_MSG(can_messages_e msg, uint32_t bus);
static status_t canfd_send_CAN_IF_MSG_BATTERY_PACK_INFO_MSG(can_messages_e msg, uint32_t bus);  
static status_t canfd_send_CAN_IF_MSG_HW_RST_FG(can_messages_e msg, uint32_t bus);  
static status_t canfd_send_CAN_IF_MSG_BMS_SET_PWR_MODE(can_messages_e msg, uint32_t bus);  
static status_t canfd_send_CAN_IF_MSG_BMS_GET_EC_AT_BOOT(can_messages_e msg, uint32_t bus);  
static status_t canfd_send_CAN_IF_MSG_RST_TAMPER_SENTINEL(can_messages_e msg, uint32_t bus); 

/* The sequence of entries in this array must match can_messages_e enumeration in can_fd_bms_if.h  */
static const can_message_t can_message_index[CAN_IF_MAX_MSG_IDS] = {
    {CAN_IF_MSG_GET_EM_INFO_ID,           0U,   42U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_GET_EM_INFO               },
    {CAN_IF_MSG_GET_TEMPERATURE_INFO_ID,  0U,   34U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_ID_GET_TEMPERATURE_INFO   },
    {CAN_IF_MSG_GET_CM_DIAG_INFO_ID,      0U,   14U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_ID_GET_CM_DIAG_INFO       },
    {CAN_IF_MSG_GET_FG_INFO_ID,           0U,   36U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_ID_GET_FG_INFO            },
    {CAN_IF_MSG_GET_CELLS_ID,             0U,   28U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_ID_GET_CELLS_INFO         },
    {CAN_IF_MSG_GET_SENSOR_INFO_ID,       0U,   32U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_ID_GET_SENSOR_INFO        },
    {CAN_IF_MSG_BMS_STATUS_ID,            0U,   8U,     0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_ID_BMS_STATUS             },
    {CAN_IF_MSG_BAL_INFO_ID,              0U,   6U,     0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_ID_BAL_INFO               },
    {CAN_IF_MSG_GET_FW_INFO_ID,           0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_GET_FW_INFO               },
    {CAN_IF_MSG_GET_LAST_RST_STATE_ID,    0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_GET_LAST_RST_STATE        },
    {CAN_IF_MSG_GET_UID_ID,               0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_GET_UID                   },
    {CAN_IF_MSG_NTF_INIT_COMPLETE_ID,     0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_NTF_INIT_COMPLETE         },
    {CAN_IF_MSG_GET_EXCP_ID,              0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_GET_EXCP                  },
    {CAN_IF_MSG_GET_SLOT_VTG_ID,          0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_GET_SLOT_VTG              },
    {CAN_IF_MSG_GET_LOG_FS_SZ_ID,         0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_GET_LOG_FS_SZ             },
    {CAN_IF_MSG_NTF_BMS_RDY_ID,           0U,   64U,    0U,   CAN_MSG_TYPE_NTF,    canfd_send_CAN_IF_MSG_NTF_BMS_RDY               },
    {CAN_IF_MSG_CM_PART2_ID,              0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_CM_PART2                  },
    {CAN_IF_MSG_BMS_HEART_BEAT_ID,        0U,   64U,    0U,   CAN_MSG_TYPE_NTF,    canfd_send_CAN_IF_MSG_BMS_HEART_BEAT            },
    {CAN_IF_MSG_BMS_READ_TIME_ID,         0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_BMS_READ_TIME             },
    {CAN_IF_MSG_BMS_WAKE_NTF_ID,          0U,   64U,    0U,   CAN_MSG_TYPE_NTF,    canfd_send_CAN_IF_MSG_BMS_WAKE_NTF              },
    {CAN_IF_MSG_DSG_CHG_FET_ON_ID,        0U,   64U,    0U,   CAN_MSG_TYPE_SET,    canfd_send_CAN_IF_MSG_DSG_CHG_FET_ON            },
    {CAN_IF_MSG_DSG_CHG_FET_OFF_ID,       0U,   64U,    0U,   CAN_MSG_TYPE_SET,    canfd_send_CAN_IF_MSG_DSG_CHG_FET_OFF           },
    {CAN_IF_MSG_DSG_FET_ON_ID,            0U,   64U,    0U,   CAN_MSG_TYPE_SET,    canfd_send_CAN_IF_MSG_DSG_FET_ON                },
    {CAN_IF_MSG_DSG_FET_OFF_ID,           0U,   64U,    0U,   CAN_MSG_TYPE_SET,    canfd_send_CAN_IF_MSG_DSG_FET_OFF               },
    {CAN_IF_MSG_CHG_FET_ON_ID,            0U,   64U,    0U,   CAN_MSG_TYPE_SET,    canfd_send_CAN_IF_MSG_CHG_FET_ON                },
    {CAN_IF_MSG_CHG_FET_OFF_ID,           0U,   64U,    0U,   CAN_MSG_TYPE_SET,    canfd_send_CAN_IF_MSG_CHG_FET_OFF               },
    {CAN_IF_MSG_BMS_EXECEPTION_INFO_ID,   0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_BMS_EXECEPTION_INFO       },
    {CAN_IF_MSG_BMS_DSG_OC_INFO_ID,       0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_BMS_DSG_OC_INFO           },
    {CAN_IF_MSG_BMS_CHG_OC_INFO_ID,       0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_BMS_CHG_OC_INFO           },
    {CAN_IF_MSG_BMS_DSG_UV_INFO_ID,       0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_BMS_DSG_UV_INFO           },
    {CAN_IF_MSG_BMS_CHG_OV_INFO_ID,       0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_BMS_CHG_OV_INFO           },
    {CAN_IF_MSG_BMS_PEAK_DSG_I_INFO_ID,   0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_BMS_PEAK_DSG_I_INFO       },
    {CAN_IF_MSG_BMS_BF_BAL_CELL_INFO_ID,  0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_BMS_BF_BAL_CELL_INFO      }, 
    {CAN_IF_MSG_BMS_AF_BAL_CELL_INFO_ID,  0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_BMS_AF_BAL_CELL_INFO      }, 
    {CAN_IF_MSG_BMS_FORCE_BAL_SHTDN_ID,   0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_BMS_FORCE_BAL_SHTDN       }, 
    {CAN_IF_MSG_BMS_FW_UPD_MSG_ID,        0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_BMS_FW_UPD_MSG            }, 
    {CAN_IF_MSG_BMS_FW_UPD_DONE_MSG_ID,   0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_BMS_FW_UPD_DONE_MSG       }, 
    {CAN_IF_MSG_BATTERY_PACK_INFO_MSG_ID, 0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_BATTERY_PACK_INFO_MSG     }, 
    {CAN_IF_MSG_BMS_RST_FG_ID,            0U,   64U,    0U,   CAN_MSG_TYPE_SET,    canfd_send_CAN_IF_MSG_HW_RST_FG                 }, 
    {CAN_IF_MSG_BMS_SET_PWR_MODE_ID,      0U,   64U,    0U,   CAN_MSG_TYPE_SET,    canfd_send_CAN_IF_MSG_BMS_SET_PWR_MODE          },
    {CAN_IF_MSG_BMS_GET_EC_AT_BOOT_ID,    0U,   64U,    0U,   CAN_MSG_TYPE_GET,    canfd_send_CAN_IF_MSG_BMS_GET_EC_AT_BOOT        }, 
    {CAN_IF_MSG_RST_TAMPER_SENTINEL_ID,   0U,   64U,    0U,   CAN_MSG_TYPE_SET,    canfd_send_CAN_IF_MSG_RST_TAMPER_SENTINEL       }, 
};

#ifdef USE_FEATURE_CAN_MSG_ID_DEBUG
static const char *msg_id_str[CAN_IF_MAX_MSG_IDS] = {
    "CAN_IF_MSG_GET_EM_INFO_ID",             
    "CAN_IF_MSG_GET_TEMPERATURE_INFO_ID",
    "CAN_IF_MSG_GET_CM_DIAG_INFO_ID",    
    "CAN_IF_MSG_GET_FG_INFO_ID",         
    "CAN_IF_MSG_GET_CELLS_ID",           
    "CAN_IF_MSG_GET_SENSOR_INFO_ID",     
    "CAN_IF_MSG_BMS_STATUS_ID",          
    "CAN_IF_MSG_BAL_INFO_ID",            
    "CAN_IF_MSG_GET_FW_INFO_ID",         
    "CAN_IF_MSG_GET_LAST_RST_STATE_ID",  
    "CAN_IF_MSG_GET_UID_ID",             
    "CAN_IF_MSG_NTF_INIT_COMPLETE_ID",   
    "CAN_IF_MSG_GET_EXCP_ID",            
    "CAN_IF_MSG_GET_SLOT_VTG_ID",        
    "CAN_IF_MSG_GET_LOG_FS_SZ_ID",       
    "CAN_IF_MSG_NTF_BMS_RDY_ID",         
    "CAN_IF_MSG_CM_PART2_ID",            
    "CAN_IF_MSG_BMS_HEART_BEAT_ID",      
    "CAN_IF_MSG_BMS_READ_TIME_ID",       
    "CAN_IF_MSG_BMS_WAKE_NTF_ID",        
    "CAN_IF_MSG_DSG_CHG_FET_ON_ID",      
    "CAN_IF_MSG_DSG_CHG_FET_OFF_ID",     
    "CAN_IF_MSG_DSG_FET_ON_ID",          
    "CAN_IF_MSG_DSG_FET_OFF_ID",         
    "CAN_IF_MSG_CHG_FET_ON_ID",          
    "CAN_IF_MSG_CHG_FET_OFF_ID",
    "CAN_IF_MSG_BMS_EXECEPTION_INFO_ID",
    "CAN_IF_MSG_BMS_DSG_OC_INFO_ID",
    "CAN_IF_MSG_BMS_CHG_OC_INFO_ID",
    "CAN_IF_MSG_BMS_DSG_UV_INFO_ID",
    "CAN_IF_MSG_BMS_CHG_OV_INFO_ID",
    "CAN_IF_MSG_BMS_PEAK_DSG_I_INFO_ID",

};
#endif /* USE_FEATURE_CAN_MSG_ID_DEBUG */

static status_t canfd_send_CAN_IF_MSG_BMS_EXECEPTION_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0xa7U, 0xb6U, 0x12U, 0x92U, 0x1fU, 0xefU, 0x20U, 0x32U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);    
    
    return s;    
}

static status_t canfd_send_CAN_IF_MSG_GET_EM_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x22U, 0xc9U, 0x0fU, 0x52U, 0x92U, 0xafU, 0x8dU, 0x9dU};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;
}

static status_t  canfd_send_CAN_IF_MSG_ID_GET_TEMPERATURE_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0xf8U, 0xbdU, 0x37U, 0xbaU, 0x5cU, 0xedU, 0x40U, 0x97U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    s = can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    if(s == STATUS_ERROR)
    {
#ifdef USE_FEATURE_CAN_MSG_ID_DEBUG
        dbg_printf("E,%d,%s send fail.\n\r", bus, msg_id_str[msg_id]);
#endif
    }

    return s;
}

static status_t  canfd_send_CAN_IF_MSG_ID_GET_CM_DIAG_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x72U, 0xacU, 0x23U, 0xd8U, 0xcbU, 0x85U, 0xe9U, 0x3aU};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;
}

static status_t  canfd_send_CAN_IF_MSG_ID_GET_FG_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x21U, 0x65U, 0x68U, 0x0dU, 0x52U, 0x58U, 0xa9U, 0xceU};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;  
}

static status_t  canfd_send_CAN_IF_MSG_ID_GET_CELLS_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x1aU, 0x8cU, 0xadU, 0x91U, 0x6aU, 0xbbU, 0x9dU, 0x5aU};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;   
}

static status_t  canfd_send_CAN_IF_MSG_ID_GET_SENSOR_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x50U, 0xf2U, 0x79U, 0xd5U, 0xbdU, 0xfbU, 0x11U, 0x49U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;   
}

static status_t  canfd_send_CAN_IF_MSG_ID_BMS_STATUS(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x47U, 0x48U, 0x97U, 0x83U, 0xd1U, 0xcfU, 0x71U, 0xe5U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;   
}

static status_t  canfd_send_CAN_IF_MSG_ID_BAL_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x6fU, 0x41U, 0xf7U, 0x33U, 0x94U, 0x0cU, 0xb3U, 0x60U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;   
}

static status_t  canfd_send_CAN_IF_MSG_GET_FW_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0xa5U, 0xcfU, 0x51U, 0xe3U, 0x96U, 0x84U, 0x08U, 0x97U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s; 
}

static status_t  canfd_send_CAN_IF_MSG_GET_LAST_RST_STATE(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x91U, 0x36U, 0x4fU, 0xedU, 0x55U, 0x67U, 0x2dU, 0xa2U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;
}

static status_t  canfd_send_CAN_IF_MSG_GET_UID(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x2bU, 0x54U, 0xabU, 0xd0U, 0x78U, 0xaeU, 0x8eU, 0xcaU};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;
}

static status_t  canfd_send_CAN_IF_MSG_NTF_INIT_COMPLETE(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0xe4U, 0xd0U, 0xb0U, 0x8dU, 0x65U, 0x84U, 0x06U, 0x94U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s; 
}

static status_t  canfd_send_CAN_IF_MSG_GET_EXCP(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x5aU, 0x75U, 0xabU, 0x70U, 0x0eU, 0x11U, 0x5cU, 0xbaU};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s; 
}

static status_t  canfd_send_CAN_IF_MSG_GET_SLOT_VTG(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x72U, 0x08U, 0x80U, 0x80U, 0x0aU, 0xd9U, 0xd4U, 0xc7U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s; 
}

static status_t  canfd_send_CAN_IF_MSG_GET_LOG_FS_SZ(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x73U, 0x80U, 0x69U, 0x95U, 0x71U, 0x7cU, 0xb4U, 0x81U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s; 
}

static status_t  canfd_send_CAN_IF_MSG_NTF_BMS_RDY(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x85U, 0xabU, 0x58U, 0xabU, 0x7aU, 0x8dU, 0x2aU, 0xa8U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s; 
}

static status_t  canfd_send_CAN_IF_MSG_CM_PART2(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0xaeU, 0x36U, 0x1dU, 0x99U, 0x10U, 0xeeU, 0xe0U, 0x02U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;  
}

static status_t  canfd_send_CAN_IF_MSG_BMS_HEART_BEAT(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x4bU, 0xe5U, 0x87U, 0x65U, 0x54U, 0xd8U, 0x7fU, 0x14U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;
}

static status_t  canfd_send_CAN_IF_MSG_BMS_READ_TIME(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x2dU, 0x71U, 0x58U, 0xc1U, 0xaeU, 0xcfU, 0x03U, 0x34U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;
}

static status_t  canfd_send_CAN_IF_MSG_BMS_WAKE_NTF(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x08U, 0x49U, 0x29U, 0xf4U, 0x49U, 0x87U, 0xf4U, 0xd0U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;
}

static status_t  canfd_send_CAN_IF_MSG_DSG_CHG_FET_ON(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0xaaU, 0x0eU, 0xd9U, 0x9dU, 0x03U, 0xfaU, 0xdfU, 0x61U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;
}

static status_t  canfd_send_CAN_IF_MSG_DSG_CHG_FET_OFF(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x14U, 0xdaU, 0x0dU, 0x68U, 0x00U, 0x70U, 0x3eU, 0x63U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;
}

static status_t  canfd_send_CAN_IF_MSG_DSG_FET_ON(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0xb2U, 0xcaU, 0x8dU, 0x8aU, 0xecU, 0x28U, 0xc6U, 0xe4U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;
}

static status_t  canfd_send_CAN_IF_MSG_DSG_FET_OFF(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x1dU, 0xefU, 0xdeU, 0x8bU, 0x67U, 0x19U, 0xdaU, 0xabU};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s; 
}

static status_t  canfd_send_CAN_IF_MSG_CHG_FET_ON(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x55U, 0xa8U, 0x90U, 0x37U, 0x1fU, 0x86U, 0xc6U, 0x20U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s; 
}

static status_t  canfd_send_CAN_IF_MSG_CHG_FET_OFF(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x42U, 0x8aU, 0x5bU, 0xffU, 0xfeU, 0x66U, 0xe7U, 0xe9U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);
    
    return s;   
}

static status_t canfd_send_CAN_IF_MSG_BMS_DSG_OC_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x7AU, 0x28U, 0x83U, 0x81U, 0xbaU, 0xdeU, 0xd2U, 0x17U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);  

    return s;       
}

static status_t canfd_send_CAN_IF_MSG_BMS_CHG_OC_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x42U, 0xd8U, 0x10U, 0x8aU, 0x2aU, 0x19U, 0xd1U, 0xc6U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);    
    
    return s;       
}

static status_t canfd_send_CAN_IF_MSG_BMS_DSG_UV_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x71U, 0x28U, 0x8dU, 0xd0U, 0xcaU, 0xd3U, 0xd7U, 0x97U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);    
    
    return s;       
}

static status_t canfd_send_CAN_IF_MSG_BMS_CHG_OV_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0xb2U, 0xd8U, 0x8aU, 0x8cU, 0x0eU, 0xd1U, 0x14U, 0xc4U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus); 

    return s;       
}

static status_t canfd_send_CAN_IF_MSG_BMS_PEAK_DSG_I_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0xc2U, 0x68U, 0x82U, 0xfcU, 0xfeU, 0xd0U, 0x10U, 0xa4U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus); 

    return s;       
}

static status_t canfd_send_CAN_IF_MSG_BMS_BF_BAL_CELL_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x42U, 0x6FU, 0x12U, 0xfcU, 0xfeU, 0xdDU, 0x1CU, 0xaeU};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus); 

    return s;       
}

static status_t canfd_send_CAN_IF_MSG_BMS_AF_BAL_CELL_INFO(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x4fU, 0x6eU, 0x1fU, 0xf9U, 0xf6U, 0xdDU, 0x16U, 0xacU};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus); 

    return s;       
}

static status_t canfd_send_CAN_IF_MSG_BMS_FORCE_BAL_SHTDN(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x21U, 0x5fU, 0x41U, 0x04U, 0x23U, 0x1eU, 0xeEU, 0xa4U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus); 

    return s;       
}

static status_t canfd_send_CAN_IF_MSG_BMS_FW_UPD_DONE_MSG(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    sys_msg_queue_obj_t smq;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x2fU, 0xffU, 0xd1U, 0x54U, 0x53U, 0xdaU, 0x5EU, 0xb4U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus); 

    osif_time_delay(1000);
    canfd_disable_bus(0U);
    
    smq.msg_id = MSG_ID_REBOOT_REQ;
    smq.msg_len = 0U;
    (void)osif_msg_queue_send(sys_msg_queue, &smq, 0U, 0U); 
    
    return s;    
}

static status_t canfd_send_CAN_IF_MSG_BMS_FW_UPD_MSG(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;

    static volatile uint32_t bms_ota_base_addr = S32_BMS_FOTA_DL_ADDR;
    int32_t lc = 0;
    int32_t i = 0;
    uint32_t j = 0;
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */

#ifdef S32K148_SERIES
    mscm_ocmdr0_save = MSCM->OCMDR[0u];
    mscm_ocmdr1_save = MSCM->OCMDR[1u];
    
    MSCM->OCMDR[0u] |= MSCM_OCMDR_OCM1(0x3u);
    MSCM->OCMDR[1u] |= MSCM_OCMDR_OCM1(0x3u);
#endif /* S32K148_SERIES */

    fw_msg_sig[0] = 0xF1U;
    fw_msg_sig[1] = 0x5AU;
    fw_msg_sig[2] = 0x4CU;
    fw_msg_sig[3] = 0x14U;
    fw_msg_sig[4] = 0xCCU;
    fw_msg_sig[5] = 0x1eU;
    fw_msg_sig[6] = 0xF8U;
    fw_msg_sig[7] = 0xC4U;
    
    /* read 40 byte block from flash and send */
    lc = osif_enter_critical();
    i = 8;
    for(j = 0U; j < BMS_FOTA_BLOCK_SIZE; j++)
    {
        fw_msg_sig[i] = *((uint8_t *)(bms_ota_base_addr + j));
        i++;
    }
    
    bms_ota_base_addr = bms_ota_base_addr + BMS_FOTA_BLOCK_SIZE;
    (void)osif_exit_critical(lc);
    
    (void)can_fd_if_bms_send(fw_msg_sig, CANFD_MSG_SIG_LEN + BMS_FOTA_BLOCK_SIZE, msg_id, bus);
    
#ifdef S32K148_SERIES
        MSCM->OCMDR[0u] = mscm_ocmdr0_save;
        MSCM->OCMDR[1u] = mscm_ocmdr1_save;
#endif /* S32K148_SERIES */

    return s;       
}

static status_t canfd_send_CAN_IF_MSG_BATTERY_PACK_INFO_MSG(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0xa1U, 0xcbU, 0x51U, 0x23U, 0x96U, 0x44U, 0x08U, 0xb7U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);  

    return s;     
}

static status_t canfd_send_CAN_IF_MSG_HW_RST_FG(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x41U, 0xc8U, 0xd0U, 0x2aU, 0xdaU, 0x1cU, 0xddU, 0xcdU};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);  

    return s;     
}

static status_t canfd_send_CAN_IF_MSG_BMS_SET_PWR_MODE(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    int lc = 0;
    
    uint8_t pwr_mode_msg[CANFD_MSG_SIG_LEN + 1U] = {0x4EU, 0xD8U, 0x10U, 0x42U, 0x4bU, 0x11U, 0x22U, 0x03U, 0x0U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */

    lc = osif_enter_critical();
    if(bms_get_charger_state() == 1U)
    {
        pwr_mode_msg[8] = 1U;
    }
    else
    {
        pwr_mode_msg[8] = 0U;
    }
    (void)osif_exit_critical(lc);
    
    (void)can_fd_if_bms_send(pwr_mode_msg, CANFD_MSG_SIG_LEN + 1U, msg_id, bus);  

    return s;     
}

static status_t canfd_send_CAN_IF_MSG_BMS_GET_EC_AT_BOOT(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x45U, 0xb8U, 0x1cU, 0x41U, 0x4bU, 0xc1U, 0xb2U, 0x01U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);   
    
    return s;         
}

static status_t canfd_send_CAN_IF_MSG_RST_TAMPER_SENTINEL(can_messages_e msg, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x33U, 0xFCU, 0xCDU, 0x44U, 0x1AU, 0xACU, 0x34U, 0x90U};
    
    can_message_t can_msg = can_message_index[msg];
    
    msg_id |= slot_to_msg_id_map[bus] | ((uint32_t)can_msg.can_msg_id << CAN_MSG_MSG_ID_SHIFT) |   \
              (can_msg.can_msg_prio << CAN_MSG_PRIO_SHIFT);
    
    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_send(msg_sig, CANFD_MSG_SIG_LEN, msg_id, bus);   
    
    return s;             
}

status_t can_wake_msg_send(uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    
    uint32_t msg_id = 0U;
    msg_id |= slot_to_msg_id_map[bus] | (CAN_IF_MSG_ID_WAKEUP_BMS_ID  << CAN_MSG_MSG_ID_SHIFT) |   \
              (0U << CAN_MSG_PRIO_SHIFT);
    
    uint8_t dummy_buffer[8] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    
    (void)can_fd_if_bms_send(dummy_buffer, 8U, msg_id, bus);
    
    return s;
}

status_t can_msg_send_process(canfd_queue_desc_t message, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    
    DEV_ASSERT(message.msg_id < CAN_IF_MAX_MSG_IDS);
    DEV_ASSERT(bus < MAX_CANFD_LOGICAL_INTERFACES);
    
#ifdef USE_FEATURE_CAN_MSG_ID_DEBUG
    dbg_printf("I,%d,UL > %d\n\r", bus, message.msg_id);
#endif
    
    can_message_index[message.msg_id].pf(message.msg_id, bus);
    if(message.is_cyclic == 1U)
    {
        (void)canfd_enqueue(message, bus);
    }
    
    return s;    
}

