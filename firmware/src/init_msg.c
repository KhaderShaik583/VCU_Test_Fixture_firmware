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
 
#include "init_msg.h" 
#include "bms.h"
#include "canfd_queue.h"

#ifdef USE_FEATURE_VCU_ON_DESK
#define NUM_BOOT_INIT_MSGS  (20U)
#else
#define NUM_BOOT_INIT_MSGS  (19U)
#endif

#define NUM_FAST_BOOT_MSGS  (4U)
static const init_msgs_t fast_boot_msgs[NUM_FAST_BOOT_MSGS] = {
    { CAN_IF_MSG_GET_FW_INFO,               0U, 0U },
    { CAN_IF_MSG_BMS_FORCE_BAL_SHTDN,       0U, 0U },
    { CAN_IF_MSG_NTF_INIT_COMPLETE,         0U, 0U },
    { CAN_IF_MSG_ID_BMS_STATUS,             0U, 0U }
};
    
static const init_msgs_t boot_msgs[NUM_BOOT_INIT_MSGS] = {
    
    { CAN_IF_MSG_BMS_SET_PWR_MODE,          0U, 0U },
    { CAN_IF_MSG_GET_BATT_INFO,             0U, 0U },
    { CAN_IF_MSG_CM_PART2,                  0U, 0U },
    { CAN_IF_MSG_BMS_EXECEPTION_INFO,       0U, 0U },
    { CAN_IF_MSG_GET_UID,                   0U, 0U },
    
#ifdef USE_FEATURE_VCU_ON_DESK
    { CAN_IF_MSG_RST_TAMPER_SENTINEL,       0U, 0U },
#endif
    
    { CAN_IF_MSG_GET_LAST_RST_STATE,        0U, 0U },
    { CAN_IF_MSG_ID_BAL_INFO,               0U, 0U },
    { CAN_IF_MSG_BMS_BF_BAL_CELL_INFO,      0U, 0U },
    { CAN_IF_MSG_BMS_AF_BAL_CELL_INFO,      0U, 0U },
    { CAN_IF_MSG_BMS_GET_EC_AT_BOOT,        0U, 0U },
    { CAN_IF_MSG_ID_GET_CM_DIAG_INFO,       0U, 0U },

    { CAN_IF_MSG_ID_GET_EM_INFO,            1U, 0U },
    { CAN_IF_MSG_ID_GET_CELLS_INFO,         1U, 0U },
    { CAN_IF_MSG_CM_PART2,                  1U, 0U },
    { CAN_IF_MSG_ID_GET_TEMPERATURE_INFO,   1U, 0U },
    { CAN_IF_MSG_ID_GET_FG_INFO,            1U, 0U },
    { CAN_IF_MSG_ID_GET_SENSOR_INFO,        1U, 0U },
    { CAN_IF_MSG_ID_BMS_STATUS,             1U, 0U },

    /* Heart beat must be the last message */
    { CAN_IF_MSG_BMS_HEART_BEAT,            1U, 0U },

};

init_msgs_t get_boot_mesg_last_mesg(void)
{
    uint32_t idx = sizeof(boot_msgs) / sizeof(boot_msgs[0]);
    
    return boot_msgs[idx - 1U];
}

void init_boot_mesg(void)
{
    canfd_queue_desc_t can_messages;
    uint32_t idx = sizeof(boot_msgs) / sizeof(boot_msgs[0]);
    uint32_t i = 0U;
    uint32_t res = QUEUE_SUCCESS;
    
    for(i = 0U; i < idx; i++)
    {
        can_messages.is_cyclic = boot_msgs[i].msg_is_loop;
        can_messages.cycle_count = boot_msgs[i].msg_loop_count;
        can_messages.msg_id = boot_msgs[i].msg_id;
        
        res = canfd_enqueue(can_messages, 0U);

        DEV_ASSERT(res == QUEUE_SUCCESS);
        
    }
}

void init_fast_boot_mesg(void)
{
    canfd_queue_desc_t can_messages;
    uint32_t idx = sizeof(fast_boot_msgs) / sizeof(fast_boot_msgs[0]);
    uint32_t i = 0U;
    uint32_t res = QUEUE_SUCCESS;
    
    for(i = 0U; i < idx; i++)
    {
        can_messages.is_cyclic = fast_boot_msgs[i].msg_is_loop;
        can_messages.cycle_count = fast_boot_msgs[i].msg_loop_count;
        can_messages.msg_id = fast_boot_msgs[i].msg_id;
        
        res = canfd_enqueue(can_messages, 0U);

        DEV_ASSERT(res == QUEUE_SUCCESS);
        
    }
}

void init_queue(uint32_t bus, uint8_t last_fet_state)
{
    canfd_queue_desc_t can_messages;
    uint32_t idx = sizeof(boot_msgs) / sizeof(boot_msgs[0]);
    uint32_t i = 0U;
    uint32_t res = QUEUE_SUCCESS;
    
    UNUSED_PARAM(last_fet_state);
    
    for(i = 0U; i < idx; i++)
    {
        can_messages.is_cyclic = boot_msgs[i].msg_is_loop;
        can_messages.cycle_count = boot_msgs[i].msg_loop_count;
        can_messages.msg_id = boot_msgs[i].msg_id;
        
        res = canfd_enqueue(can_messages, (uint8_t)bus);

        DEV_ASSERT(res == QUEUE_SUCCESS);
        
    }    
}
