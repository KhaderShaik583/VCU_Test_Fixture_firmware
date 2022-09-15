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
 
#include "version.h" 

static const char *fw_date = __DATE__;
static const char *fw_timestamp = __TIME__;
static const char *fw_name = "UV_VCU";
static const uint8_t fw_version_max = 18U;
static const uint8_t fw_version_min = 11U;

const char *get_fw_date(void)
{
    return fw_date;
}

const char *get_fw_timestamp(void)
{
    return fw_timestamp;
}

uint8_t get_fw_max_ver(void)
{
    return fw_version_max;
}

uint8_t get_fw_min_ver(void)
{
    return fw_version_min;
}
