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
 
#include "odometer_task.h"
#include "can_messenger_rx.h"
#include "dba_task.h"
#include "mc_task.h"
#include "shared_mem.h"

#define NUM_KM_BEFORE_NVM_UPDATE    (0.5f)
#define ODO_UPDATE_RESOLUTION_KM    (0.1f)
#define NUM_KM_BEFORE_RANGE_UPDATE  (1.0f)

static volatile odometer_persistence_t odo_current_nvm;
static float_t watt_hours[NVM_MAX_TRIPS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static float_t odo_value_at_load = 0.0f;

static float_t wh_per_km = 0.0f;
static float_t wh_prev = 0.0f;

static float_t wh_per_km_rg = 0.0f;
static float_t wh_prev_rg = 0.0f;
static float_t wh_rg_initial = 0.0f;

static float_t av_cap_at_poweron = 0.0f;
static float_t batt_voltage_at_poweron = 0.0f;
static float_t wh_at_poweron = 0.0f;

/* 1D-Kalman filter state variables */
static float_t q = 0.000125f;
static float_t r = 200.0f;
static float_t p = 0.000125f;
static float_t k = 0.0f;
static float_t x = 0.0f;

#ifdef USE_RANGE_PRED_ALG0
static volatile uint16_t pred_range = 200U;

static float_t range_kalman_filt(float_t range)
{
    volatile float_t estimated_range = 0.0f;
    
    p = p + q;
    
    if(((p + r) > 0.0f) &&
       ((p + r) != NAN))
    {
        k = p / (p + r);
    }
    else
    {
        k = 0.0f;
    }
    
    x = x + k * (range - x);
    p = (1.0f - k) * p;
    
    estimated_range = x;
    
    return estimated_range;    
}

static void range_at_start(float_t wh)
{   
    /* Using unused trip wh field for storing last ride Wh/km */   
    if((odo_current_nvm.trip[3].watt_hour > 0.0f) &&
       (odo_current_nvm.trip[3].watt_hour != NAN))
    {
        pred_range = (uint16_t)(wh / odo_current_nvm.trip[3].watt_hour);
        r = pred_range;
    }
    else
    {
        pred_range = 200U;
    }

    if(pred_range > 200U)
    {
        pred_range = 200U;
    }
    
    x = pred_range;
}

static float_t wh_running_avg(float_t wh_km)
{
    static uint32_t sample_count = 0U;
    static float_t wh_avg = 0.0f;
    
    sample_count++;
    
    if(sample_count > 0U)
    {
        wh_avg = (float_t)(((sample_count - 1.0f) / sample_count) * wh_avg) + ((1.0f / sample_count) * wh_km);
    }
    
    return wh_avg;    
}

static void range_sm(void)
{
    volatile float_t bms_av_cap = 0U;
    volatile float_t wh = 0.0f;
    volatile float_t bms_pack_ocv = 0.0f;

    bms_get_avcap(0U, (float_t *)&bms_av_cap);
    bms_get_pack_ocv(0U, (float_t *)&bms_pack_ocv);
    
    wh = bms_av_cap * bms_pack_ocv;
    
    if((wh_per_km - wh_per_km_rg) > 0.0f)
    {
        pred_range = (uint16_t)(wh / (wh_per_km - wh_per_km_rg));
    }
    
    if(pred_range > 200U)
    {
        pred_range = 200U;
    }
    
#if 0
    if(pred_range > 0U)
    {
        pred_range = (uint16_t)range_kalman_filt(pred_range);
    }
    else
    {
        pred_range = 0U;
    }
#endif /* 0 */
}

static void wh_sm(void)
{
    float_t wh = 0.0f;
    float_t wh_rg = 0.0f;
    
    bms_get_dsg_energy_wh(0U, &wh);
    bms_get_chg_energy_wh(0U, &wh_rg);
    
    if(wh_prev == 0.0f)
    {
        wh_per_km = wh - watt_hours[4];
    }
    else
    {
        wh_per_km = wh - wh_prev;
    }
    
    if(wh_rg > wh_rg_initial)
    {
        if(wh_prev_rg == 0.0f)
        {
            wh_per_km_rg = wh_rg - wh_rg_initial;
        }
        else
        {
            wh_per_km_rg = wh_rg - wh_prev_rg;
        }
    }
    
    wh_prev = wh;
    wh_prev_rg = wh_rg;
    
    /* Using unused trip wh field for storing last ride Wh/km */
    if(wh_per_km > wh_per_km_rg)
    {
        odo_current_nvm.trip[3].watt_hour = wh_running_avg(wh_per_km - wh_per_km_rg);
    }
}  

static void update_range(float_t distance)
{
    static float_t distance_to_update = 0.0f;
    
    distance_to_update = distance_to_update + distance;

    if(distance_to_update > NUM_KM_BEFORE_RANGE_UPDATE)
    {
        distance_to_update = 0.0f;
        range_sm();
        wh_sm();
    }
}
#endif /* USE_RANGE_PRED_ALG0 */

static void update_trip_counters(void)
{
    volatile uint32_t mc_avg_speed = 0U;
    volatile uint32_t time_since_boot = 0U;
    
    if(odo_current_nvm.odometer > odo_value_at_load)
    {
        mc_avg_speed =  mc_get_vehicle_average_speed();
        time_since_boot = osif_millis();
        
        /* Vehicle supports only three trips */
        odo_current_nvm.trip[0].average_speed = mc_avg_speed;
        odo_current_nvm.trip[1].average_speed = mc_avg_speed;
        odo_current_nvm.trip[2].average_speed = mc_avg_speed;
        
        odo_current_nvm.trip[3].average_speed = 0.0f;

        /* 4 is used for current trip */
        odo_current_nvm.trip[4].average_speed = mc_avg_speed;

    }
}

static void trip_reset(uint32_t trip_num)
{
    int32_t lc = 0;

    lc = osif_enter_critical();
    
    odo_current_nvm.trip[trip_num].distance         = 0.0f;
    odo_current_nvm.trip[trip_num].watt_hour        = 0.0f;
    odo_current_nvm.trip[trip_num].trip_duration    = 0.0f;
    odo_current_nvm.trip[trip_num].average_speed    = 0.0f;
    
    (void)osif_exit_critical(lc);
}

#ifdef USE_RANGE_PRED_ALG0
void odo_get_range(uint16_t *range_in_km)
{
    *range_in_km = pred_range;
}

void odo_get_wh_quantized(float_t *wh_pk)
{
    *wh_pk = wh_per_km;
}

void odo_get_wh_rg_quantized(float_t *wh_pk)
{
    *wh_pk = wh_per_km_rg;
}
#endif /* USE_RANGE_PRED_ALG0 */

void update_trip_watthrs(float_t dsg_energy, float_t chg_energy)
{
    static uint32_t state = 0U;
    
    UNUSED_PARAM(chg_energy);
    
    if(state == 0U)
    {
        /* First update */
        watt_hours[0] = dsg_energy / (float_t)3600;
        watt_hours[1] = dsg_energy / (float_t)3600;
        watt_hours[2] = dsg_energy / (float_t)3600;
        watt_hours[3] = 0.0f;
        watt_hours[4] = dsg_energy / (float_t)3600;
        
        bms_get_chg_energy_wh(0U, &wh_rg_initial);
        bms_get_avcap(0U, (float_t *)&av_cap_at_poweron);
        bms_get_voltage_cm(&batt_voltage_at_poweron);
        
        wh_at_poweron = av_cap_at_poweron * batt_voltage_at_poweron;
        
        range_at_start(wh_at_poweron);
        
        state = 1U;
    }
    else if(state == 1U)
    {
        odo_current_nvm.trip[0].watt_hour += (dsg_energy / (float_t)3600) - watt_hours[0];
        odo_current_nvm.trip[1].watt_hour += (dsg_energy / (float_t)3600) - watt_hours[1];
        odo_current_nvm.trip[2].watt_hour += (dsg_energy / (float_t)3600) - watt_hours[2];

        odo_current_nvm.trip[4].watt_hour = (dsg_energy / (float_t)3600) - watt_hours[4]; 
    }
    else
    {
        __NOP();
    }
}

float_t odo_get_distance(void)
{
    return odo_current_nvm.odometer;
}

/**
 * @brief - Returns the distance covered in the current trip
            
 * @param - none
 * @return - trip[4] distance in km 
 */

float_t odo_get_trip_distance(void)
{
    return odo_current_nvm.trip[4].distance;
}

void odo_get_trip_info(trip_info_t *trip_data, uint32_t trip_num)
{
    int32_t lc = 0;
    
	DEV_ASSERT(trip_data != NULL);
	
    if(trip_num < MAX_USR_RESETTABLE_TRIPS)
    {
        lc = osif_enter_critical();
        
		trip_data->distance = odo_current_nvm.trip[trip_num].distance;
		trip_data->watt_hour = odo_current_nvm.trip[trip_num].watt_hour;
		trip_data->trip_duration = odo_current_nvm.trip[trip_num].trip_duration;
		trip_data->average_speed = odo_current_nvm.trip[trip_num].average_speed;
        
        (void)osif_exit_critical(lc);
	}
	else
	{
		__NOP();
	}
}

int32_t odo_get_info(uint8_t *buffer)
{
    int32_t n = 0;
    int32_t lc = 0;
    
    DEV_ASSERT(buffer != NULL);
    
    update_trip_counters();

    lc = osif_enter_critical();
    
    /* snprintf does not count null so +1 */ 
    /* Format: Odo Value At Load, Odo, Trip1, Trip2, Trip3, Current Trip
       Trips: Distance (km), Wh, duration (sec), average speed (kmph)
    */
    n = 1 + snprintf((char *)buffer, SHMEM_BMS_BLK_SIZE, "%.2f,%.2f|"
                                "%.2f,%.2f,%.2f,%.2f|"
                                "%.2f,%.2f,%.2f,%.2f|"
                                "%.2f,%.2f,%.2f,%.2f|"
                                "%.2f,%.2f,%.2f,%.2f\n", 
    
        odo_value_at_load,
		odo_current_nvm.odometer,
		odo_current_nvm.trip[0].distance,
		odo_current_nvm.trip[0].watt_hour,
		odo_current_nvm.trip[0].trip_duration,
		odo_current_nvm.trip[0].average_speed,

		odo_current_nvm.trip[1].distance,
		odo_current_nvm.trip[1].watt_hour,
		odo_current_nvm.trip[1].trip_duration,
		odo_current_nvm.trip[1].average_speed,
    
		odo_current_nvm.trip[2].distance,
		odo_current_nvm.trip[2].watt_hour,
		odo_current_nvm.trip[2].trip_duration,
		odo_current_nvm.trip[2].average_speed,

		odo_current_nvm.trip[4].distance,
		odo_current_nvm.trip[4].watt_hour,
		odo_current_nvm.trip[4].trip_duration,
		odo_current_nvm.trip[4].average_speed);

    (void)osif_exit_critical(lc);

    return n;
}

static void odo_trip_time_sm(float_t distance)
{
    static uint32_t sm_state = 0U;
    static volatile uint32_t start_time = 0U;
    static uint32_t end_time = 0U;
    
    switch(sm_state)
    {
        case 0U:
            if(distance > 0.0f)
            {
                start_time = osif_millis();
                sm_state = 1U;
            }
            else
            {
                sm_state = 0U;
            }
            break;
       
        case 1U:
            if(distance == 0.0f)
            {
                end_time = osif_millis();
                
                odo_current_nvm.trip[0].trip_duration += (float_t)((end_time - start_time) / 1000U);
                odo_current_nvm.trip[1].trip_duration += (float_t)((end_time - start_time) / 1000U);
                odo_current_nvm.trip[2].trip_duration += (float_t)((end_time - start_time) / 1000U);
                
                odo_current_nvm.trip[4].trip_duration += (float_t)((end_time - start_time) / 1000U);
                
                sm_state = 0U;
            }
            else
            {
                sm_state = 1U;
            }
            break;
    }            
}

/**
 * @brief - Odometer and trip calculation. Called periodically from the 
 *          motor controller or abs module.

            trip[0], trip[1] and trip[2] are primary trip meters.
            trip[3] is unused may be used as a scratchpad.
            trip[4] is the current trip.
            
 * @param distance - Distance travelled in the given time period.
 * @return void 
 */

void update_counters(float_t distance)
{
    int32_t lc = 0;
    uint8_t persistent_hash[CMAC_HASH_SIZE];

    /* distance_to_update is the distance in km after which the data will be 
       flushed to the NVM 
    */
    static volatile float_t distance_to_update_nvm = 0.0f;
    static volatile float_t distance_to_update_odo = 0.0f;
	static uint32_t last_time_update = 0U;
	static volatile uint32_t time_since_start = 0U;
    
    lc = osif_enter_critical();
    
    distance_to_update_odo += distance;
    distance_to_update_nvm += distance;
 
    if(distance_to_update_odo > ODO_UPDATE_RESOLUTION_KM)
    {
        distance_to_update_odo = 0.0f;
        odo_current_nvm.odometer += ODO_UPDATE_RESOLUTION_KM;
        
        odo_current_nvm.trip[0].distance += ODO_UPDATE_RESOLUTION_KM;
        odo_current_nvm.trip[1].distance += ODO_UPDATE_RESOLUTION_KM;
        odo_current_nvm.trip[2].distance += ODO_UPDATE_RESOLUTION_KM;
        odo_current_nvm.trip[4].distance += ODO_UPDATE_RESOLUTION_KM;
        
        time_since_start = osif_millis();
        if((time_since_start - last_time_update) > 100U)
        {
            odo_current_nvm.trip[0].trip_duration += (float_t)((time_since_start - last_time_update) / 1000U);
            odo_current_nvm.trip[1].trip_duration += (float_t)((time_since_start - last_time_update) / 1000U);
            odo_current_nvm.trip[2].trip_duration += (float_t)((time_since_start - last_time_update) / 1000U);
            
            odo_current_nvm.trip[4].trip_duration += (float_t)((time_since_start - last_time_update) / 1000U);
            
            last_time_update = osif_millis();
        }
    }
    
    (void)osif_exit_critical(lc);
    
    if(distance_to_update_nvm > NUM_KM_BEFORE_NVM_UPDATE)
    {
        /* Every NUM_KM_BEFORE_UPDATE km update the persistence store with the odometer reading */
        distance_to_update_nvm = 0.0f;

        /* Write Current NVM */
        (void)nvm_write(FILE_SECT_ODO_PERSISTENCE, (uint8_t *)&odo_current_nvm, sizeof(odometer_persistence_t));

        /* In the event of a sudden power loss beyond this point, the MAC may not get updated anyways. update_odo_persistence
           could update the MAC or init_odometer in the next power cycle. 
        */
#if 0
        /* Compute MAC */
        aes_cmacnvm((uint8_t *)&odo_current_nvm, sizeof(odometer_persistence_t), persistent_hash);
            
        /* Place the MAC at the end of the sector. */
        (void)nvm_write((FILE_SECT_ODO_PERSISTENCE + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE); 
#endif
        
        __DMB();
        __DSB();
    }

    update_range(distance);
}

void update_odo_persistence(void)
{
    uint8_t persistent_hash[CMAC_HASH_SIZE];

    /* Write Current NVM */
    (void)nvm_write(FILE_SECT_ODO_PERSISTENCE, (uint8_t *)&odo_current_nvm, sizeof(odometer_persistence_t));
        
    /* Compute MAC */
    aes_cmacnvm((uint8_t *)&odo_current_nvm, sizeof(odometer_persistence_t), persistent_hash);
        
    /* Place the MAC at the end of the sector. */
    (void)nvm_write((FILE_SECT_ODO_PERSISTENCE + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);   
    
    __DMB();
    __DSB();
}

void odo_reset(void)
{
    int32_t lc = 0;
    
    lc = osif_enter_critical();
    
    (void)memset((uint8_t *)&odo_current_nvm, 0U, sizeof(odometer_persistence_t));
    odo_current_nvm.sentinel = 0xFFFFFFFFU;
    update_odo_persistence();
    init_odometer();
    
    (void)osif_exit_critical(lc);
}

void odo_trip_reset(uint32_t trip_num)
{
    /* VCU supports 5 trip meters, uses only 4, three of which are resettable */
    if(trip_num < MAX_USR_RESETTABLE_TRIPS)
    {
        trip_reset(trip_num);
    }
}

void init_odometer(void)
{
    uint32_t sentinel_addr = 0U;
    uint8_t persistent_hash[CMAC_HASH_SIZE];
    uint8_t calc_persistent_hash[CMAC_HASH_SIZE];
    int32_t macval = 1;

    volatile uint32_t persistent_nvm_sz = sizeof(odometer_persistence_t);
    
    DEV_ASSERT(persistent_nvm_sz < (NVM_FILE_SECTOR_BYTES - 20U));
    
    nvm_read(FILE_SECT_ODO_PERSISTENCE, (uint8_t *)&sentinel_addr, sizeof(uint32_t));
    
    if(sentinel_addr != 0xDEADCAFEU)
    {
        (void)memset((uint8_t *)&odo_current_nvm, 0U, sizeof(odometer_persistence_t));
        odo_current_nvm.sentinel = 0xDEADCAFEU;
        
        /* First time configuration */
        (void)nvm_write(FILE_SECT_ODO_PERSISTENCE, (uint8_t *)&odo_current_nvm, sizeof(odometer_persistence_t));

        /* Compute MAC */
        aes_cmacnvm((uint8_t *)&odo_current_nvm, sizeof(odometer_persistence_t), persistent_hash);
        
        /* Place the MAC at the end of the sector. Leaving 4 Byte guard */
        (void)nvm_write((FILE_SECT_ODO_PERSISTENCE + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
        
        odo_value_at_load = 0.0f;
    }
    else
    {
        /* Load NVM */
        nvm_read(FILE_SECT_ODO_PERSISTENCE, (uint8_t *)&odo_current_nvm, sizeof(odometer_persistence_t));

        /* Load hash */
        nvm_read((FILE_SECT_ODO_PERSISTENCE + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
        
        /* Compute MAC */
        aes_cmacnvm((uint8_t *)&odo_current_nvm, sizeof(odometer_persistence_t), calc_persistent_hash);
        
        macval = memcmp(calc_persistent_hash, persistent_hash, CMAC_KEY_SIZE); 
        
        /* Signal NVM Integrity */
        if(macval != 0)
        {
            /* Do not reset the odometer. Instead calculate the hash and update the NVM. */
            
            /* Place the MAC at the end of the sector. Leaving 4 Byte guard */
            (void)nvm_write((FILE_SECT_ODO_PERSISTENCE + (NVM_FILE_SECTOR_BYTES - 20U)), calc_persistent_hash, CMAC_KEY_SIZE);

            set_status_bit(STAT_VCU_ODO_NVM_ERROR);
        }
        
        odo_value_at_load = odo_current_nvm.odometer;
        trip_reset(TRIP_NUM_4);
    }
}

