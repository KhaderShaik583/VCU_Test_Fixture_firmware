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
 
#ifndef INIT_MSG_H
#define INIT_MSG_H

#include "fw_common.h" 

typedef struct
{
    uint32_t msg_id;
    uint32_t msg_is_loop;
    int32_t msg_loop_count;
}init_msgs_t;

void init_boot_mesg(void);
void init_fast_boot_mesg(void);
void init_queue(uint32_t pack, uint8_t last_fet_state);
init_msgs_t get_boot_mesg_last_mesg(void);

#endif /* INIT_MSG_H */
