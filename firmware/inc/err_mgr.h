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
 
#ifndef ERR_MGR_H
#define ERR_MGR_H

#include <stdint.h>
#include "fw_common.h"

typedef enum {
    SYS_ERR_PERIPH_INIT_LPSPI0 = 0U,
    SYS_ERR_PERIPH_INIT_LPSPI1,
    SYS_ERR_PERIPH_INIT_LPSPI2,
    SYS_ERR_PERIPH_INIT_LPI2C0,
    SYS_ERR_PERIPH_INIT_LPI2C1,
    SYS_ERR_PERIPH_INIT_FTM,
    SYS_ERR_PERIPH_INIT_MSDI

}sys_err_type_e;


void err_mgr_set_err(sys_err_type_e err);
uint64_t err_mgr_get_err(void);

static inline void err_notifier(volatile bool x, sys_err_type_e e)
{
	if(x) { 
        
    } 
    else {
        err_mgr_set_err(e);
    }
}
#define ERR_NOTIFY(x, y) err_notifier(x, y)

#endif /* ERR_MGR_H */
