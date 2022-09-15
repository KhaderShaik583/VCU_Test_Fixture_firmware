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
 
#include "sem_common.h"
#include "osif.h"
#include "rtx_os.h"

static semaphore_t lpi2c_bus_sem_t;

status_t lpi2c_sem_acquire(uint32_t timeout)
{
    status_t s;
    s = osif_sem_acquire(&lpi2c_bus_sem_t, timeout);
    return s;
}

status_t lpi2c_sem_release(void)
{
    status_t s;
    s = osif_sem_release(&lpi2c_bus_sem_t);
    return s;
}

void sem_common_init(void)
{
    /* Common semaphore for the I2C bus accesses */
    status_t status = STATUS_ERROR;
    
    status = osif_sem_create(&lpi2c_bus_sem_t, 0U);
    DEV_ASSERT(status == STATUS_SUCCESS);
    
}
