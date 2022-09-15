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
 
#ifndef SWIF_TASK_H
#define SWIF_TASK_H

#include "fw_common.h"

#define SWIF_TASK_NAME           "thread_swif"
#define SWIF_TASK_STACK_SIZE     STACK_SIZE(128U)

/* Thread Flags */
#define SWIF_TASK_START_FLAG     (0x0001U)

void swif_task_create(void);
osThreadId_t swif_task_get_id(void);

#endif /* SWIF_TASK_H */ 
