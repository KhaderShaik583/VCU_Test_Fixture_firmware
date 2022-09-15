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
 
#ifndef CAN_MESSENGER_RX_H
#define CAN_MESSENGER_RX_H

#include "fw_common.h"

#include "flexcan_driver.h"
#include "canfd_queue.h"
#include "batterypack.h"
#include "odometer_task.h"

typedef struct
{
    uint8_t api_version[4];
    uint32_t msg_sequence;
    uint32_t millis;
    uint32_t statush;
    uint32_t statusl;
    uint32_t vcu_statush;
    uint32_t vcu_statusl;
    float_t roll;
    float_t pitch;
    float_t odometer;
    dev_uid_t bms_id;
    uint8_t throttle_voltage[4];
    uint8_t speed[4];
    uint8_t actual_speed[4];
    uint8_t distance[4];
    uint8_t vcu_id[16];
    float_t wh_per_km;
    float_t wh_per_km_regen;
    uint32_t available_modes;
    uint32_t current_ride_mode;
    uint32_t vehicle_range_type;
    uint16_t range;
    uint8_t rtc[8];
    uint8_t bus;
}vcu_info_msg_t;


typedef struct AuxiliaryInfo {
    uint32_t meas_marker;
    float_t dsg_ov_value;
    float_t chg_ov_value;
    uint16_t swif_internal_faults;
    uint32_t als_lux;
    uint8_t reserved[26];
}auxiliary_info_t;

typedef struct
{
    uint32_t soc;
    float_t pack_voltage;
    float_t pack_current;
    float_t max_cell_temperature;
    float_t max_cell_voltage;
    float_t min_cell_voltage;
    float_t motor_temperature;
    float_t motor_heatsink_temperature;
    float_t fet_temperature;
    int32_t shaft_rpm;
    uint32_t available_modes;
}imx_dbg_msg_t;

status_t process_bms_can_data(const uint8_t *buffer, flexcan_msgbuff_t bcu_can_msg, uint16_t *tx_len, uint32_t bus);
void can_notify_rx_process(canfd_queue_desc_t canfd_message, uint32_t bus);
void debug_bms_data(uint32_t bus);
void can_enqueue_msg(uint32_t bus);
void fet_sw_init(void);
uint32_t get_dsg_fet_state(uint32_t bus);
uint32_t get_chg_fet_state(uint32_t bus);
void create_fet_on_timer(void);
void can_fd_if_bms_err_process(uint32_t *esr);
void bms_state_init(void);
void bms_create_tx_timer(void);
void bms_data_tx_timer_stop(void);
void bms_data_tx_timer_start(uint32_t duration);
float_t bms_get_soc(uint32_t bus);
void bms_get_tte(uint32_t bus, uint32_t *tte);
float_t bms_get_max_temp(uint32_t bus);
char *bms_get_fw_version(void);
void bms_get_balancer_info(balancer_run_info_t *bal_info);
void bms_get_dsg_energy_wh(uint32_t bus, float_t *dsg_energy);
void bms_get_chg_energy_wh(uint32_t bus, float_t *chg_energy);
void bms_get_chg_ah(uint32_t bus, uint32_t *chg_ah);
void bms_get_vi(float_t *v, float_t *i, ltc2946_channels_e ch);
void bms_get_design_capacity(uint8_t *design_cap);
void bms_fg_reset(void);
uint32_t bms_get_variant_info(void);
uint8_t bms_get_range_type(void);
float_t bms_get_min_cell_voltage(uint32_t bus);
void bms_get_avcap(uint32_t bus, float_t *avcap);
void bms_get_pack_ocv(uint32_t bus, float_t *pack_estimated_ocv);
void bms_get_current(float_t *i, ltc2946_channels_e ch);
void bms_get_voltage_cm(float_t *v);
void bms_get_max_cell_temperature(float_t *cell_t);
#endif /* CAN_MESSENGER_RX_H */
