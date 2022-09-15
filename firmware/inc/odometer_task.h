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
 
#ifndef ODOMETER_TASK_H
#define ODOMETER_TASK_H

#include "fw_common.h"
#include "csec_driver.h"
#include "aes.h"
#include "nvm.h"

/*  VCU supports 5 trip meters. Meters 0, 1 and 2 are user resettable, 3 may/may not be 
    used or used as scratchpad and 4 is used as the current trip meter which resets at every boot
*/
#define MAX_USR_RESETTABLE_TRIPS  (3U)

typedef enum
{
    TRIP_NUM_0 = 0,
    TRIP_NUM_1,
    TRIP_NUM_2,
    TRIP_NUM_3,
    TRIP_NUM_4,
    
    MAX_TRIP_COUNT
    
}trip_count_e;


typedef struct 
{
    trip_info_t trip[MAX_USR_RESETTABLE_TRIPS];
}trip_meter_disp_t;

void init_odometer(void);
void update_odo_persistence(void);
void update_counters(float_t distance);
void update_trip_watthrs(float_t wh_d, float_t wh_ch);
int32_t odo_get_info(uint8_t *buffer);
float_t odo_get_distance(void);
float_t odo_get_trip_distance(void);
void odo_reset(void);
void odo_trip_reset(uint32_t trip_num);
void odo_get_range(uint16_t *range_in_km);
void odo_get_wh_quantized(float_t *wh_pk);
void odo_get_wh_rg_quantized(float_t *wh_pk);
void odo_get_trip_info(trip_info_t *trip_data, uint32_t trip_num);
#endif /* ODOMETER_TASK_H */
