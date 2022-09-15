#ifndef CHARGER_TASK_H
#define CHARGER_TASK_H

#include "board.h"
#include "fw_common.h"

#define CHG_TASK_NAME            "thread_chg"
#define CHG_TASK_STACK_SIZE      STACK_SIZE(512U)

/* Thread Flags */
#define CHG_TASK_START_FLAG             (0x0001U)
#define CHG_TASK_WAIT_DISCONNECT_FLAG   (0x0004U)

#define CHG_MSGQUEUE_OBJECTS            (10U) 

void chg_task_create(void);
thread_id_t chg_task_get_id(void);

#define CHG_MSG_BOOT_NTF_RSP_ID         (0x119U)
#define CHG_MSG_CHG_SET_IOUT_ID         (0x11AU)
#define CHG_MSG_CHRG_FAULTS_ID			(0x11BU)
#define CHG_MSG_BATT_VI_ID     			(0x11CU)
#define CHG_MSG_BATT_INT_RESIST_ID		(0x11DU)
#define CHG_MSG_CHARGER_ON_OFF_CTRL     (0x11EU)
#define CHG_MSG_CHG_BOOT_NTF_ID         (0x11FU)
#define CAN_MSG_CHG_HBT_ID              (0x121U)
#define CAN_MSG_ANALOG_MEAS_ID          (0x131U)

#define CHG_MQ_MSG_FAULT_INFO           (0x1BU)
#define CHG_MQ_MSG_BATT_IR              (0x1DU)
#define CHG_MQ_MSG_CHARGER_CTRL         (0x1EU)
#define CHG_MQ_MSG_ANALOG_MEAS          (0x1CU)

#define CHARGER_TIMEOUT_PERIOD_MS       (2000U)

#define CHARGER_CTRL_OFF                (0xC0U)
#define CHARGER_CTRL_ON                 (0xC1U)

/*
    OBC - On-Board Charger.
    EXT - External 3 KW chargers from Ultraviolette.
    STN - EVSE / Charging station.
    PSU - External power supply unit unauthorized.
*/
#define CHARGER_DEV_TYPE_OBC            (0xCAU)
#define CHARGER_DEV_TYPE_EXT            (0xCBU)
#define CHARGER_DEV_TYPE_STN            (0xCDU)
#define CHARGER_DEV_TYPE_PSU            (0xCFU)

typedef enum
{
    CHARGER_SM_WAIT_TASK_START_NTF   = 0U,
    CHARGER_SM_WAIT_CHARGER_BOOT_NTF,
    CHARGER_SM_SEND_NTF,
    CHARGER_SM_SEND_CHARGE_CURRENT,
    CHARGER_SM_CHECK_KEY_STATE,
    CHARGER_SM_WAIT_KEY_OFF,
    CHARGER_SM_START_CHARGING,
    CHARGER_SM_WAIT_BATT_VOLTAGE,
    CHARGER_SM_SEND_BATT_VI,
    CHARGER_SM_CHECK_BATT_TEMPERATURES,
    CHARGER_SM_WAIT_BATT_COOLDOWN,
    CHARGER_SM_CHECK_FAULTS,
    CHARGER_SM_CHECK_C20_CURRENT,
    CHARGER_SM_CHECK_KEY_STATE2,
    CHARGER_SM_CLEANUP,
    CHARGER_WAIT_DISCONNECT,
    
    CHARGER_SM_MAX_STATES
}charger_states_e;


typedef enum
{
    SYS_CHARGING_IN_PROGRESS = 0U,
    SYS_FAN1_FAIL_FAULT,            /* Fault bit for FAN1 failure(LOCK1) */
	SYS_PSFBMOSFET_OVERTEMP_FAULT,  /* Fault bit for PSFB MOSFET over temperature Failure */
	SYS_TRANSFORMER_OVERTEMP_FAULT, /* Fault bit for PSFB transformer over temperature Failure*/
	SYS_PFCMOSFET_OVERTEMP_FAULT,   /* Fault bit for PFC MOSFET over temperature Failure*/
	SYS_OVLOAD_CV_FAULT,            /* Fault bit for output overcurrent */
	SYS_OVLOAD_CC_FAULT,            /* Fault bit for output overvoltage */	
	SYS_AC_UVP_FAULT,               /* Fault bit for AC_UVP i.e undervoltage protection */
	SYS_AC_OVP_FAULT,               /* Fault bit for AC_OVP overvoltage  protection*/
    SYS_LINE_FAULT,                 /* Fault bit for interchange in Line and Neutral event */
	SYS_OVP_FAULT,                  /* Fault bit for over voltage */
	SYS_PFC_FAULT,                  /* Fault bit for PFC fail */
    SYS_CC_CURRENT_FAULT,           /* Fault bit for when VCU sends Iout to set is out of range */
	SYS_VCU_COMMUNICATION_FAIL,     /* Fault bit for indicating CAN communication failure with VCU */ 
	SYS_VCU_AUTH_FAIL,              /* Fault bit for indicating authentication failure from VCU */
	SYS_CC_CV_COMMAND_TIMEOUT,      /* Fault bit for not receiving cc_cv settings from VCU withing 5sec time*/
	SYS_CHRG_TIMEOUT,               /* Fault bit if overall charging timeout */
	SYS_PFCTEMP_SENSEFAIL,          /* PFC temperature sensor failed */
	SYS_PSFBTEMP_SENSEFAIL,         /* PSFB temperature sensor failed */
	SYS_TRANSFORMER_TEMP_FAIL,      /* Diode temperature sensor failed */
	SYS_HEARTBEAT_FAULT,			/* VCU hearbeat timeout error */
	SYS_WAKEUP_TIMEOUT_FAULT,		/* VCU wakeup timeout error */
	
    MAX_CHG_STATUS_FLAGS           
    
}charger_status_e;

typedef struct 
{
    uint32_t msg_id;
    uint8_t data[8];
}chg_msg_queue_obj_t;

typedef struct
{
    uint32_t charger_log_boundary;
    uint32_t charger_connection_state;
    uint32_t charger_status;
    uint32_t charger_type;
    uint32_t charger_fw_major_num;
    uint32_t charger_fw_minor_num;
    uint32_t coulombs_before_charge;
    uint32_t coulombs_after_charge;
    float_t chg_bandgap_volts;
    float_t psfb_fet_temp;
    float_t transformer_temp;
    float_t pfc_fet_temp;
    float_t vofb_volts;
    float_t iofb_volts;
    osif_timer_id_t charger_comm_timer;
}charger_ctx_t;

extern osif_msg_queue_id_t chg_msg_queue;
uint32_t chg_get_connection_state(void);
uint32_t chg_get_key_pos(void);
uint32_t charger_get_status(void);
void chg_get_charger_context(charger_ctx_t *charger_ctxt);

#endif /* CHARGER_TASK_H */
