#ifndef BATTERY_PACK_H
#define BATTERY_PACK_H

#include "fw_common.h"
#include "erm_driver.h"

#define CELL_CHANNELS                   (14U)
#define AUX_CHANNELS                    (10U)
#define MAX_TEMP_AUX_SENSORS            (18U)
#define MAX_SYS_SENSORS                 (11U)

#define LTC2946_ALERT_MIN_ADIN          (0x00)
#define LTC2946_ALERT_MAX_ADIN          (0x01)
#define LTC2946_ALERT_MIN_VIN           (0x02)
#define LTC2946_ALERT_MAX_VIN           (0x03)
#define LTC2946_ALERT_MIN_ISENSE        (0x04)
#define LTC2946_ALERT_MAX_ISENSE        (0x05)
#define LTC2946_ALERT_MIN_PWR           (0x06)
#define LTC2946_ALERT_MAX_PWR           (0x07)

#define MOSFET_ON                       (1U)
#define MOSFET_OFF                      (0U)

#define MOSFET_ON_SW_STATE              (0x0AU)
#define MOSFET_OFF_SW_STATE             (0x05U)
#define MOSFET_BOOT_SW_STATE            (0x0BU)

#define CHG_MOSFET                      (1U)
#define DSG_MOSFET                      (3U)

typedef enum
{
    /* Status Low Bits */
    STAT_DSG_FET_STATUS_FLAG = 0U,
    STAT_CHG_FET_STATUS_FLAG,
    STAT_BAL_TIMER_STATUS_FLAG,
    STAT_BAL_ACT_STATUS_FLAG,
    STAT_LTC2946_DSG_ALERT_FLAG,
    STAT_LTC2946_CHG_ALERT_FLAG,
    STAT_PWR_MODE_CHARGE,
    STAT_PWR_MODE_CHARGE_NX,
    STAT_BMS_UNECOVERABLE_FAILURE,
    STAT_UV_THR_FLAG,
    STAT_OV_THR_FLAG,
    STAT_LTC6812_WDT_SET_FLAG,
    STAT_BATTERY_TEMP_OVER_MIN_THRESHOLD,
    STAT_BATTERY_TEMP_OVER_MAX_THRESHOLD,
    STAT_BATTERY_TEMP_TOO_LOW,
    STAT_LTC6812_SAFETY_TIMER_FLAG,
    STAT_BALANCER_ABORT_FLAG,
    STAT_BALANCER_RESET_FLAG,
    STAT_BALANCING_COMPLETE_FLAG,
    STAT_LTC6812_PEC_ERROR,
    STAT_UV_OV_THR_FOR_TURN_ON,
    STAT_ECC_ERM_ERR_FLAG,
    STAT_DSG_INA302_ALERT1,
    STAT_DSG_INA302_ALERT2,
    STAT_MOSFET_OVER_TMP_ALERT,
    STAT_FET_FRONT_OVER_TMP_ALERT,
    STAT_BAT_PLUS_OVER_TMP_ALERT,
    STAT_BAT_MINUS_OVER_TMP_ALERT,
    STAT_PACK_PLUS_MCPCB_OVER_TMP_ALERT,
    STAT_REL_HUMIDITY_OVERVALUE_ALERT,
    STAT_DSG_FUSE_BLOWN_ALERT,
    STAT_CHG_FUSE_BLOWN_ALERT,
    STAT_FET_TURN_ON_FAILURE,
    STAT_FET_TURN_OFF_FAILURE,
    
    /* Status High Bits */
    STAT_BAL_RES_OVER_TEMPERATURE,
    STAT_LTC2946_COMM_FAILURE,
    STAT_HW_UV_SHUTDOWN,
    STAT_HW_OV_SHUTDOWN,
    STAT_HW_OVER_TMP_SHUTDOWN,
    STAT_LTC7103_PGOOD,
    STAT_SYS_BOOT_FAILURE,
    STAT_CAN_MSG_SIG_ERR,
    STAT_FG_I2C_BUS_RECOVERY_EXEC,
    STAT_FG_MEAS_ABORT,
    STAT_BAT_TAMPER_DETECTED,
    STAT_TMP_THR_FOR_TURN_ON,
    STAT_FET_FRONT_OVER_TMP_WARN,
    STAT_BAT_PLUS_OVER_TMP_WARN,
    STAT_BAT_MINUS_OVER_TMP_WARN,
    STAT_PACK_PLUS_OVER_TMP_WARN,
    STAT_FG_MEAS_ERROR,
    STAT_PM_CHG_CURRENT_LIMIT_UPDATE,
    STAT_BIT_UNUSED10,
    STAT_BIT_UNUSED11,
    STAT_BIT_UNUSED12,
    STAT_BIT_UNUSED13,
    STAT_BIT_UNUSED14,
    STAT_BIT_UNUSED15,
    STAT_BIT_UNUSED16,
    STAT_BIT_UNUSED17,
    STAT_BIT_UNUSED18,
    STAT_BIT_UNUSED19,
    STAT_BIT_UNUSED20,
    
    MAX_BMS_STATUS_FLAGS
    
}pack_status_e;


typedef enum {
    LTC2946_DSG_CHANNEL = 0,
    LTC2946_CHG_CHANNEL,
    MAX_LTC_CHANNELS
}ltc2946_channels_e;

typedef struct {

    uint8_t alert;
    uint8_t fault;
    uint8_t status;
    uint16_t swstatus;
    float_t current;
    float_t vin;
    float_t charge;
    float_t time;
    float_t adin;
    float_t sc_current;
    float_t energy;
    float_t power;

}ltc2946_meas_t;

typedef struct {
    uint16_t current_code;
    uint16_t vin_code;
    uint16_t adin_code;
    uint32_t charge_code;
    uint32_t time_code;
    uint32_t energy_code;
}ltc2946_raw_meas_t;

typedef struct {
    uint32_t cell_idx;
    uint16_t cell_voltage;
}ov_uv_cell_info_t;

typedef struct {

    uint8_t alert;
    uint8_t fault;
    uint8_t status;
    uint16_t swstatus;
    float_t current;
    float_t time;
    float_t adin;
    float_t charge;
}ltc2946_meas_min_t;

typedef struct {
    uint16_t cv[CELL_CHANNELS];
    int16_t t[MAX_TEMP_AUX_SENSORS];
    float_t mosfet_temp;
    uint16_t aux_vref;
    uint16_t min_cv;
    uint16_t soc;
    uint16_t BalanceTime;
    uint16_t cd_bitmap;
}ltc6812_meas_t;

typedef struct {
    uint16_t itmp;
    uint16_t va;
    uint16_t vd;
    uint16_t mux_fail;
    uint16_t adc_selftest_fail;
    uint16_t cell_index;
    uint32_t open_cell;
}ltc6812_diag_t;     

typedef struct {
    uint16_t min_cv;
    uint16_t cd_bitmap;
}ltc6812_min_meas_t;

typedef struct
{
    float_t voltage;
    float_t vbatt;
    float_t avg_current;
    float_t soc;
    float_t tte;
    float_t ttf;
    float_t ts;
    float_t repcap;
    float_t internal_resistance;
    uint16_t num_cycles;
    uint16_t fstat;
    uint8_t designCapacity;
}max17205_ctx_t;

typedef struct {
    uint32_t meas_idx;
    float_t meas_value;    
}max17205_indexed_meas_t;

/* 264 bytes no-pack */
typedef struct BMSMeasurements {
    ltc2946_meas_t LTC2946[2];
    ltc6812_meas_t LTC6812;
    float_t sensor_values[6];
    float_t delta_voltage;
    float_t dsg_oc_value;
    float_t chg_oc_value;
    float_t dsg_uv_cell_value;
    float_t chg_uv_cell_value;
    uint32_t dsg_uv_cell_index;
    uint32_t chg_uv_cell_index;
    float_t peak_currents[3];
    uint32_t can_esr;
    max17205_ctx_t m;
}bms_meas_t;

typedef struct BalancerRunInfo
{
    uint32_t balancer_run_count;
    uint16_t cv_before_balancing[CELL_CHANNELS];
    uint16_t cv_after_balancing[CELL_CHANNELS];
    uint32_t balancer_time_ticks;  
    uint32_t balancer_abort_cause;
}balancer_run_info_t;

typedef struct 
{
    uint32_t uidl;
    uint32_t uidml;
    uint32_t uidmh;
    uint32_t uidh;
}dev_uid_t;

typedef struct 
{
    erm_ecc_event_t ecc_evt;
    uint32_t address;
}erm_ecc_err_t;


typedef struct
{
    uint8_t channela_ctrla;
    uint8_t channela_ctrlb;
    uint8_t channela_alert1;
    uint8_t channela_gpiocfg;
    uint8_t channela_clkdiv;
    uint8_t channelb_ctrla;
    uint8_t channelb_ctrlb;
    uint8_t channelb_alert1;
    uint8_t channelb_gpiocfg;
    uint8_t channelb_clkdiv;
}ltc2946_dev_settings_t;

typedef struct 
{
    float_t channela_max_current_threshold;
    float_t channela_min_adin_threshold;
    float_t channela_max_adin_threshold;
    float_t channelb_max_current_threshold;
    float_t channelb_max_adin_threshold;
    float_t ltc2946_adin_divider_ratio;
}ltc2946_dev_config_t;

typedef struct
{
    uint16_t ov_uv_safety_timeout;
    uint16_t balancing_wait_time;
    uint16_t predsg_timer_timeout;
    float_t bal_temp_threshold;

    float_t min_soc;
    float_t max_soc;

    float_t max_temperature_warning_threshold;
    float_t max_temperature_threshold;
    float_t min_temperature_threshold;
    float_t mosfet_temperature_max_threshold;
    float_t min_temperature_for_fet_turn_on;
    
    float_t ts_bot_threshold;
    float_t ts_top_threshold;
    float_t pwr_con_n_threshold;
    float_t pwr_con_p_threshold;
}bms_parameters_t;

typedef struct 
{
    uint8_t theoretical_capacity;
    uint8_t num_cells_in_series;
    uint8_t num_cells_in_parallel;
    uint8_t battery_chemistry;
    uint8_t cell_manufacturer;
    uint8_t nominal_ah;
    uint8_t range_type;
}bms_info_t; 
    
typedef struct 
{
    uint16_t max17205_packcfg;
    uint16_t max17205_ve;
    uint16_t max17205_vr;
    uint16_t max17205_designcap;
    uint16_t max17250_cfg;
}max17205_parameters_t;

typedef struct
{
    uint32_t sentinel;
    ltc2946_dev_settings_t dev_ltc2946;
    ltc2946_dev_config_t cfg_ltc2946;
    bms_parameters_t bms_params;
    bms_info_t bms_info;
    max17205_parameters_t maxim_params;  
}persistent_nvm_t;

typedef struct {
    uint32_t slot;
    uint16_t bmct;
    uint16_t bsocv;
    uint32_t bchgv;
    uint32_t bchgi;
    uint32_t bdsgi;
    uint32_t btime;
    uint64_t bsh;
    uint8_t bsoc;
    uint8_t bdsgah;
    uint8_t bchgah;
    uint8_t bdsgenergy;
    uint8_t bchgenergy;
}bms_unique_msg_t;

typedef struct {
    uint32_t slot;
    float_t vdsg;
    float_t vchg;
    float_t idsg;
    float_t ichg;
    float_t dsgq;
    float_t chgq;
    float_t timestmp;
    uint32_t statusl;
    uint32_t statush;
    float_t soc;
    float_t fettmp;
    float_t max_cell_temp;
    uint32_t sequence;
}lte_data_tx_t;

typedef struct {
    uint8_t slot;
    float_t vdsg;
    float_t vchg;
    float_t idsg;
    float_t ichg;
    float_t dsgq;
    float_t chgq;
    uint32_t statusl;
    uint32_t statush;
    float_t soc;
    float_t fettmp;
    float_t max_cell_temp;
    float_t avg_cell_voltage;
    float_t min_cell_voltage;
    float_t max_cell_voltage;
    float_t tte;
    float_t ttf;
}bms_imx_info_t;

/* 464 Bytes - 17/05/2021 */
typedef struct BatteryPackContext
{
    uint64_t status;
    uint32_t time;
    uint8_t battery_dfet_state;
    uint8_t battery_cfet_state;
    uint8_t is_balancer_running;
	dev_uid_t uid;
	bms_meas_t bms_data;
	ltc6812_diag_t bms_diag;
	balancer_run_info_t bms_balancer_ctx;
    char fw_version[64];
    uint8_t rcm_srs0;
    uint8_t rcm_srs1;
    float_t delta_v;
    uint32_t exeption_lr;
}bms_ctx_t;

typedef struct {
    float_t charge[MAX_LTC_CHANNELS];
    float_t energy[MAX_LTC_CHANNELS];
}ltc2946_boot_chg_eng_t; 

#endif /* BATTERY_PACK_H */

